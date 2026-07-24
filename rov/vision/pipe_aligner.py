"""
Boru girisi hizalama modulu.

Boru (silindirik) girisi kamera cercevesinde Hough Circle Transform veya
kontur bounding-box ile bulunur. Cikti olarak araç merkezinden piksel
farklari (x_error, y_error) dondurulur.

Kullanim:
    aligner = PipeAligner()
    result = aligner.process(frame)
    # result.x_error: + = boru sagda (yaw sagla)
    # result.y_error: + = boru asagida (daha derin in)
    # result.found: True/False
    # result.distance_score: 0-1 (1 = cok yakin / buyuk boru)
"""
import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Optional

from vision.enhance import enhance
from config import PIPE_MIN_RADIUS, PIPE_ALIGN_TOL, CAM_WIDTH, CAM_HEIGHT


@dataclass
class PipeResult:
    found: bool = False
    x_error: float = 0.0       # piksel, + = sag
    y_error: float = 0.0       # piksel, + = asagi
    radius_px: float = 0.0     # tespit edilen yaricap
    distance_score: float = 0.0 # 0-1 (1 = tam yakinda)
    aligned: bool = False       # hata tolerans icinde mi?
    debug_frame: Optional[np.ndarray] = field(default=None, repr=False)


class PipeAligner:
    """
    Boru girisini gri tonlamayi Hough Circle ile arar.
    Boru bulunamazsa kontur bounding-box'a fallback yapar.
    """
    _HOUGH_DP   = 1.2
    _HOUGH_DIST = 100   # daireler arasi min mesafe
    _HOUGH_P1   = 100   # Canny yuksek esik
    _HOUGH_P2   = 30    # akumulator esik (dusuk = daha duyarli)

    def process(self, frame: np.ndarray) -> PipeResult:
        if frame is None:
            return PipeResult()

        enhanced = enhance(frame)
        h, w = enhanced.shape[:2]
        cx_ref, cy_ref = w // 2, h // 2

        gray = cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        debug = frame.copy()
        cv2.circle(debug, (cx_ref, cy_ref), 5, (0, 255, 0), 2)  # merkez nokta

        # ── Hough Circle
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=self._HOUGH_DP,
            minDist=self._HOUGH_DIST,
            param1=self._HOUGH_P1,
            param2=self._HOUGH_P2,
            minRadius=PIPE_MIN_RADIUS,
            maxRadius=max(PIPE_MIN_RADIUS, w // 3),
        )

        if circles is not None:
            circles = np.round(circles[0, :]).astype(int)
            # En buyuk cember (en yakin boru)
            best = max(circles, key=lambda c: c[2])
            bx, by, br = best
            x_err = float(bx - cx_ref)
            y_err = float(by - cy_ref)
            dist_score = min(1.0, br / (w // 4))
            aligned = (abs(x_err) < PIPE_ALIGN_TOL and
                       abs(y_err) < PIPE_ALIGN_TOL)

            cv2.circle(debug, (bx, by), br, (0, 0, 255), 2)
            cv2.circle(debug, (bx, by), 4, (255, 0, 0), -1)
            cv2.putText(debug,
                        f"boru x={x_err:+.0f} y={y_err:+.0f} r={br}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)

            return PipeResult(
                found=True, x_error=x_err, y_error=y_err,
                radius_px=float(br), distance_score=dist_score,
                aligned=aligned, debug_frame=debug,
            )

        # ── Fallback: en buyuk kontur bounding circle
        _, thresh = cv2.threshold(blurred, 0, 255,
                                  cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            (bx, by), br = cv2.minEnclosingCircle(cnt)
            bx, by, br = int(bx), int(by), int(br)
            if br >= PIPE_MIN_RADIUS:
                x_err = float(bx - cx_ref)
                y_err = float(by - cy_ref)
                aligned = (abs(x_err) < PIPE_ALIGN_TOL and
                           abs(y_err) < PIPE_ALIGN_TOL)
                return PipeResult(
                    found=True, x_error=x_err, y_error=y_err,
                    radius_px=float(br),
                    distance_score=min(1.0, br / (w // 4)),
                    aligned=aligned, debug_frame=debug,
                )

        cv2.putText(debug, "boru bulunamadi", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return PipeResult(debug_frame=debug)
