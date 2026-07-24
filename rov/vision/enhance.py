"""
Su alti goruntu iyilestirme.

Pipeline (KTR'deki algoritma):
  1. BGR -> LAB: A/B kanal ortalama saptmasini duzelt (renk dengesi)
  2. L kanalina CLAHE: kontrast iyilestirme
  3. Geri donusturup Unsharp Masking (USM): keskinlestirme
  4. HSV'de doygunluk artirma: renk canlandirma

Kullanim:
    enhanced = enhance(frame)  # BGR girdi, BGR cikti
"""
import cv2
import numpy as np


def enhance(frame: np.ndarray, clip_limit: float = 2.0,
            tile_grid: tuple = (8, 8), usm_amount: float = 1.2,
            sat_boost: float = 1.3) -> np.ndarray:
    """
    Su alti karesi iyilestir.

    frame      : BGR numpy array
    clip_limit : CLAHE clip limiti (yuksek = daha fazla kontrast, gurultu riski)
    tile_grid  : CLAHE blok boyutu
    usm_amount : Unsharp Masking guclenme miktari (0=yok, 2=cok)
    sat_boost  : HSV doygunluk carpani (1.0=degisim yok)

    Donus: BGR numpy array
    """
    if frame is None:
        return None

    # ── 1. LAB renk dengesi ─────────────────────────────────────────────────
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
    # A ve B kanallarinin ortalamasini 128'e (notr) yaklaştır
    lab[:, :, 1] -= (lab[:, :, 1].mean() - 128.0)
    lab[:, :, 2] -= (lab[:, :, 2].mean() - 128.0)
    lab = np.clip(lab, 0, 255).astype(np.uint8)

    # ── 2. CLAHE (L kanalinda) ──────────────────────────────────────────────
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid)
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    frame_balanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # ── 3. Unsharp Masking ─────────────────────────────────────────────────
    blur = cv2.GaussianBlur(frame_balanced, (0, 0), sigmaX=3)
    sharp = cv2.addWeighted(frame_balanced, 1.0 + usm_amount, blur, -usm_amount, 0)

    # ── 4. HSV doygunluk artirma ────────────────────────────────────────────
    hsv = cv2.cvtColor(sharp, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat_boost, 0, 255)
    result = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    return result
