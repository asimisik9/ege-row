"""
Hat takip modulu — HSV maskeleme + kontur merkezleme.

Desteklenen modlar (config.py'den LINE_TRACK_MODE):
  1. "RED_BLACK" (TEKNOFEST): Kırmızı şerit içindeki siyah çizgiyi takip eder.
     - Önce kırmızı bant maskelenir (H: 0-12 ve 168-180 iki parçalı maske).
     - Kırmızı bant içindeki siyah iç çizgi tespit edilir.
     - Siyah çizgi silikse kırmızı bandın merkezine düşer (yedekli / çift korumalı).
  2. "SINGLE_HSV": Tek bir HSV renk aralığı maskeler (örn. beyaz hat).

Kullanim:
    tracker = LineTracker()
    result = tracker.process(frame)  # BGR numpy array
"""
import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Optional

from vision.enhance import enhance
from config import (LINE_TRACK_MODE, RED_HSV_LOW1, RED_HSV_HIGH1,
                    RED_HSV_LOW2, RED_HSV_HIGH2, BLACK_HSV_LOW, BLACK_HSV_HIGH,
                    LINE_HSV_LOW, LINE_HSV_HIGH, LINE_MIN_AREA, LINE_BLUR_K)


@dataclass
class LineResult:
    found: bool = False
    error_px: float = 0.0        # pozitif = hat sagda (sag don gerek)
    confidence: float = 0.0      # 0-1
    centroid_x: int = 0
    centroid_y: int = 0
    debug_frame: Optional[np.ndarray] = field(default=None, repr=False)


class LineTracker:
    """
    Tek nesne — durumsuz (stateless). Her kareyi bagimsiz isler.
    Hat maskeleme parametreleri config.py'den gelir.
    """

    def __init__(self):
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (7, 7)  # morfolojik kapama icin
        )

    def process(self, frame: np.ndarray) -> LineResult:
        """
        frame: BGR goruntu (sensors.camera.Camera'dan).
        Donus: LineResult.
        """
        if frame is None:
            return LineResult()

        # ── Goruntu iyilestirme
        enhanced = enhance(frame)
        h, w = enhanced.shape[:2]
        cx_ref = w // 2   # referans merkez

        # ── Gaussian blur (gurultu azaltma)
        blurred = cv2.GaussianBlur(
            enhanced,
            (LINE_BLUR_K, LINE_BLUR_K), 0
        )
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        debug = frame.copy()
        cv2.line(debug, (cx_ref, 0), (cx_ref, h), (0, 255, 0), 1)  # merkez cizgisi

        if LINE_TRACK_MODE == "RED_BLACK":
            return self._process_red_black(hsv, debug, h, w, cx_ref)
        else:
            return self._process_single_hsv(hsv, debug, h, w, cx_ref)

    def _process_red_black(self, hsv, debug, h, w, cx_ref) -> LineResult:
        """
        TEKNOFEST Kırmızı zemin içi siyah çizgi algılama.
        """
        # 1. Kırmızı koridor maskesi (Kırmızı H: 0-12 ve 168-180 birleşimi)
        mask_r1 = cv2.inRange(hsv, np.array(RED_HSV_LOW1, dtype=np.uint8),
                                  np.array(RED_HSV_HIGH1, dtype=np.uint8))
        mask_r2 = cv2.inRange(hsv, np.array(RED_HSV_LOW2, dtype=np.uint8),
                                  np.array(RED_HSV_HIGH2, dtype=np.uint8))
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self._kernel)

        # 2. Siyah iç çizgi maskesi
        black_mask = cv2.inRange(hsv, np.array(BLACK_HSV_LOW, dtype=np.uint8),
                                      np.array(BLACK_HSV_HIGH, dtype=np.uint8))

        # 3. Kırmızı koridor İÇİNDEKİ siyah çizgi (bitwise AND)
        inner_black_mask = cv2.bitwise_and(black_mask, red_mask)
        inner_black_mask = cv2.morphologyEx(inner_black_mask, cv2.MORPH_CLOSE, self._kernel)

        # Öncelik 1: Kırmızı zemin içindeki siyah çizgiyi bul
        contours_black, _ = cv2.findContours(inner_black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        target_contour = None
        mode_label = "Siyah Cizgi (Kirmizi Ici)"

        if contours_black:
            cnt = max(contours_black, key=cv2.contourArea)
            if cv2.contourArea(cnt) >= (LINE_MIN_AREA / 3.0):  # Siyah çizgi daha incedir
                target_contour = cnt

        # Öncelik 2 (Yedek): Siyah çizgi silikse doğrudan kırmızı bandın merkezine kilitlen
        if target_contour is None:
            contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_red:
                cnt = max(contours_red, key=cv2.contourArea)
                if cv2.contourArea(cnt) >= LINE_MIN_AREA:
                    target_contour = cnt
                    mode_label = "Kirmizi Bant (Yedek)"

        if target_contour is None:
            cv2.putText(debug, "HAT BULUNAMADI", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return LineResult(debug_frame=debug)

        # Kontur merkezini hesapla
        area = cv2.contourArea(target_contour)
        M = cv2.moments(target_contour)
        if M["m00"] == 0:
            return LineResult(debug_frame=debug)

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        error_px = cx - cx_ref

        max_area = w * h * 0.05
        confidence = min(1.0, area / max_area)

        # Debug gösterimi
        cv2.drawContours(debug, [target_contour], -1, (0, 0, 255), 2)
        cv2.circle(debug, (cx, cy), 8, (0, 255, 255), -1)
        cv2.putText(debug, f"{mode_label}  err={error_px:+.0f}px  conf={confidence:.2f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return LineResult(
            found=True,
            error_px=float(error_px),
            confidence=confidence,
            centroid_x=cx,
            centroid_y=cy,
            debug_frame=debug,
        )

    def _process_single_hsv(self, hsv, debug, h, w, cx_ref) -> LineResult:
        """
        Klasik tekli HSV maskeleme.
        """
        low  = np.array(LINE_HSV_LOW,  dtype=np.uint8)
        high = np.array(LINE_HSV_HIGH, dtype=np.uint8)
        mask = cv2.inRange(hsv, low, high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return LineResult(debug_frame=debug)

        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < LINE_MIN_AREA:
            return LineResult(debug_frame=debug)

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            return LineResult(debug_frame=debug)

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        error_px = cx - cx_ref

        max_area = w * h * 0.05
        confidence = min(1.0, area / max_area)

        cv2.drawContours(debug, [cnt], -1, (0, 0, 255), 2)
        cv2.circle(debug, (cx, cy), 8, (255, 0, 0), -1)
        cv2.putText(debug, f"err={error_px:+.0f}px  conf={confidence:.2f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        return LineResult(
            found=True,
            error_px=float(error_px),
            confidence=confidence,
            centroid_x=cx,
            centroid_y=cy,
            debug_frame=debug,
        )
