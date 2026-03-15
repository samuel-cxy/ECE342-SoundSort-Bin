# pip install numpy scipy scikit-learn matplotlib

import os
from pathlib import Path
import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score

# =========================================================
# Configuration
# =========================================================

# Root folder containing your class subfolders:
# data/tissue, data/can, data/plastic, data/other
DATA_DIR = Path("data")

# Fixed class order.
# This order matters because class index 0/1/2/3 will map to these names.
CLASSES = ["tissue", "can", "plastic", "other"]

# All audio will be resampled to this sample rate
TARGET_SR = 16000

# After trimming silence, each clip will be forced to this duration
# by either cutting or zero-padding
CLIP_DURATION_S = 0.50

# FFT size used for frequency-domain features
N_FFT = 1024

# Frequency bands for band-energy features
# Since TARGET_SR = 16000, Nyquist frequency is 8000 Hz
BAND_EDGES = [0, 300, 700, 1200, 2000, 3000, 4500, 6500, 8000]

# Random seed for reproducibility
RNG = 42


# =========================================================
# Audio loading / preprocessing
# =========================================================

def load_wav_mono(path, target_sr=16000):
    """
    Load a WAV file, convert it to mono, convert to float32,
    and resample it if necessary.

    Returns:
        sr : int
            Sample rate after processing
        x : np.ndarray
            Mono audio signal in float32
    """
    sr, x = wavfile.read(path)

    # Convert integer audio to float32 in roughly [-1, 1]
    if x.dtype == np.int16:
        x = x.astype(np.float32) / 32768.0
    elif x.dtype == np.int32:
        x = x.astype(np.float32) / 2147483648.0
    elif x.dtype == np.uint8:
        # Unsigned 8-bit PCM is usually centered at 128
        x = (x.astype(np.float32) - 128.0) / 128.0
    else:
        # If already float, just cast to float32
        x = x.astype(np.float32)

    # Convert stereo to mono by averaging channels
    if x.ndim == 2:
        x = np.mean(x, axis=1)

    # Resample to target sample rate if needed
    if sr != target_sr:
        x = resample_poly(x, target_sr, sr)
        sr = target_sr

    return sr, x


def trim_silence(x, threshold=0.03):
    """
    Remove leading and trailing low-amplitude regions.

    Args:
        x : audio signal
        threshold : amplitude threshold for detecting activity

    Returns:
        Trimmed audio signal
    """
    idx = np.where(np.abs(x) > threshold)[0]

    # If no sample exceeds threshold, return original signal
    if len(idx) == 0:
        return x

    # Keep from first active sample to last active sample
    return x[idx[0]:idx[-1] + 1]


def fix_length(x, sr, duration_s):
    """
    Force audio clip to a fixed duration.

    If x is longer than target length -> truncate
    If x is shorter -> zero-pad

    Returns:
        Fixed-length signal
    """
    target_len = int(sr * duration_s)

    if len(x) >= target_len:
        return x[:target_len]

    y = np.zeros(target_len, dtype=np.float32)
    y[:len(x)] = x
    return y


# =========================================================
# Feature extraction
# IMPORTANT:
# These features should match what you plan to compute on STM32
# =========================================================

def extract_features(x, sr):
    """
    Extract a compact feature vector from one audio clip.

    Current features:
    - RMS energy
    - Peak amplitude
    - Zero-crossing rate
    - Decay ratio
    - Spectral centroid
    - Spectral rolloff
    - Active ratio
    - Band-energy features

    Returns:
        feats : np.ndarray of shape [num_features]
    """

    # -------- Time-domain features --------

    # RMS (root-mean-square) energy
    rms = np.sqrt(np.mean(x**2) + 1e-12)

    # Peak absolute amplitude
    peak = np.max(np.abs(x)) + 1e-12

    # Zero-crossing rate:
    # measures how often the signal changes sign
    zcr = np.mean(np.abs(np.diff(np.signbit(x).astype(np.int32))))

    # -------- Simple decay feature --------
    # Compare early energy and late energy to capture "ringing" or "decay"
    third = len(x) // 3
    e1 = np.sum(x[:third] ** 2) + 1e-12
    e3 = np.sum(x[-third:] ** 2) + 1e-12
    decay_ratio = np.log(e1 / e3)

    # -------- Frequency-domain features --------

    # Apply a Hann window before FFT to reduce spectral leakage
    window = np.hanning(len(x))

    # Real FFT because input is real-valued audio
    X = np.fft.rfft(x * window, n=N_FFT)

    # Power spectrum
    P = np.abs(X) ** 2

    # Corresponding frequency axis
    freqs = np.fft.rfftfreq(N_FFT, d=1.0 / sr)

    total_energy = np.sum(P) + 1e-12

    # Compute normalized band energies
    band_energies = []
    for lo, hi in zip(BAND_EDGES[:-1], BAND_EDGES[1:]):
        mask = (freqs >= lo) & (freqs < hi)
        band_e = np.sum(P[mask]) / total_energy
        band_energies.append(band_e)

    # Spectral centroid = weighted average frequency
    spectral_centroid = np.sum(freqs * P) / total_energy

    # Spectral rolloff = frequency below which 85% of spectral energy lies
    cumsum = np.cumsum(P)
    rolloff_idx = np.searchsorted(cumsum, 0.85 * total_energy)
    spectral_rolloff = freqs[min(rolloff_idx, len(freqs) - 1)]

    # Fraction of samples above a small threshold
    active_ratio = np.mean(np.abs(x) > 0.05)

    # Final feature vector
    feats = np.array([
        rms,
        peak,
        zcr,
        decay_ratio,
        spectral_centroid,
        spectral_rolloff,
        active_ratio,
        *band_energies
    ], dtype=np.float32)

    return feats


# =========================================================
# Dataset loading
# =========================================================

def load_dataset(data_dir):
    """
    Walk through all class folders, load every WAV file,
    preprocess it, extract features, and build X/y arrays.

    Returns:
        X     : [num_samples, num_features]
        y     : [num_samples]
        paths : list of file paths
    """
    X = []
    y = []
    paths = []

    # Loop over each class folder in fixed class order
    for class_idx, class_name in enumerate(CLASSES):
        class_dir = data_dir / class_name

        if not class_dir.exists():
            print(f"Warning: missing folder {class_dir}")
            continue

        wav_files = sorted(class_dir.glob("*.wav"))
        print(f"{class_name}: {len(wav_files)} files")

        for wav_path in wav_files:
            # Load and standardize sampling rate / channels
            sr, x = load_wav_mono(wav_path, TARGET_SR)

            # Remove quiet leading / trailing regions
            x = trim_silence(x, threshold=0.03)

            # Make every sample same length
            x = fix_length(x, sr, CLIP_DURATION_S)

            # Extract hand-crafted features
            feats = extract_features(x, sr)

            # Store sample
            X.append(feats)
            y.append(class_idx)
            paths.append(str(wav_path))

    return np.array(X), np.array(y), paths


# =========================================================
# Export trained model to a C header for STM32
# =========================================================

def export_c_header(model, out_path="soundsort_model.h"):
    """
    Export scaler mean/std and logistic regression weights/bias
    to a C header file so STM32 can run inference.

    The exported arrays include:
    - FEATURE_MEAN
    - FEATURE_SCALE
    - LR_WEIGHTS
    - LR_BIAS
    """
    scaler = model.named_steps["scaler"]
    clf = model.named_steps["clf"]

    means = scaler.mean_
    scales = scaler.scale_
    W = clf.coef_       # Shape: [num_classes, num_features]
    b = clf.intercept_  # Shape: [num_classes]

    num_classes, num_features = W.shape

    with open(out_path, "w") as f:
        f.write("#ifndef SOUNDSORT_MODEL_H\n")
        f.write("#define SOUNDSORT_MODEL_H\n\n")

        f.write(f"#define NUM_CLASSES {num_classes}\n")
        f.write(f"#define NUM_FEATURES {num_features}\n\n")

        # Export class names in same order as CLASSES list
        f.write("static const char *CLASS_NAMES[NUM_CLASSES] = {")
        f.write(", ".join([f"\"{c}\"" for c in CLASSES]))
        f.write("};\n\n")

        # Export feature normalization mean
        f.write("static const float FEATURE_MEAN[NUM_FEATURES] = {\n    ")
        f.write(", ".join([f"{v:.8f}f" for v in means]))
        f.write("\n};\n\n")

        # Export feature normalization scale (std dev)
        f.write("static const float FEATURE_SCALE[NUM_FEATURES] = {\n    ")
        f.write(", ".join([f"{v:.8f}f" for v in scales]))
        f.write("\n};\n\n")

        # Export logistic regression weight matrix
        f.write("static const float LR_WEIGHTS[NUM_CLASSES][NUM_FEATURES] = {\n")
        for row in W:
            f.write("    {")
            f.write(", ".join([f"{v:.8f}f" for v in row]))
            f.write("},\n")
        f.write("};\n\n")

        # Export logistic regression bias vector
        f.write("static const float LR_BIAS[NUM_CLASSES] = {\n    ")
        f.write(", ".join([f"{v:.8f}f" for v in b]))
        f.write("\n};\n\n")

        f.write("#endif\n")

    print(f"Exported STM32 header to: {out_path}")


# =========================================================
# Main training flow
# =========================================================

def main():
    # Load feature matrix X and label vector y
    X, y, paths = load_dataset(DATA_DIR)

    print(f"\nDataset shape: X={X.shape}, y={y.shape}")

    # Stop early if no files were found
    if len(X) == 0:
        raise RuntimeError("No WAV files found.")

    # -----------------------------------------------------
    # Split dataset into:
    # 60% train, 20% validation, 20% test
    # -----------------------------------------------------

    # First split: train+val vs test
    X_trainval, X_test, y_trainval, y_test = train_test_split(
        X,
        y,
        test_size=0.20,
        random_state=RNG,
        stratify=y
    )

    # Second split: train vs val
    X_train, X_val, y_train, y_val = train_test_split(
        X_trainval,
        y_trainval,
        test_size=0.25,   # 25% of 80% = 20% of full dataset
        random_state=RNG,
        stratify=y_trainval
    )

    print(f"Train: {len(X_train)}, Val: {len(X_val)}, Test: {len(X_test)}")

    # -----------------------------------------------------
    # Build pipeline:
    # 1) Standardize features
    # 2) Train multinomial logistic regression
    # -----------------------------------------------------
    pipe = Pipeline([
        ("scaler", StandardScaler()),
        ("clf", LogisticRegression(
            multi_class="multinomial",
            solver="lbfgs",
            max_iter=3000,
            class_weight="balanced"
        ))
    ])

    # -----------------------------------------------------
    # Hyperparameter search:
    # Tune regularization strength C
    # Smaller C = stronger regularization
    # Larger C = weaker regularization
    # -----------------------------------------------------
    param_grid = {
        "clf__C": [0.01, 0.1, 1.0, 3.0, 10.0, 30.0, 100.0]
    }

    grid = GridSearchCV(
        pipe,
        param_grid=param_grid,
        cv=5,
        scoring="accuracy",
        n_jobs=-1
    )

    # Train models for all candidate C values
    grid.fit(X_train, y_train)

    best_model = grid.best_estimator_
    print("\nBest params:", grid.best_params_)

    # -----------------------------------------------------
    # Evaluate on validation set
    # -----------------------------------------------------
    y_val_pred = best_model.predict(X_val)
    val_acc = accuracy_score(y_val, y_val_pred)
    print(f"Validation accuracy: {val_acc:.4f}")

    # -----------------------------------------------------
    # Train final model on train+validation combined
    # using the best C found above
    # -----------------------------------------------------
    final_model = Pipeline([
        ("scaler", StandardScaler()),
        ("clf", LogisticRegression(
            multi_class="multinomial",
            solver="lbfgs",
            max_iter=3000,
            class_weight="balanced",
            C=grid.best_params_["clf__C"]
        ))
    ])

    final_model.fit(X_trainval, y_trainval)

    # -----------------------------------------------------
    # Final test evaluation
    # -----------------------------------------------------
    y_test_pred = final_model.predict(X_test)
    test_acc = accuracy_score(y_test, y_test_pred)

    print(f"\nTest accuracy: {test_acc:.4f}")

    print("\nClassification report:")
    print(classification_report(y_test, y_test_pred, target_names=CLASSES))

    print("Confusion matrix:")
    print(confusion_matrix(y_test, y_test_pred))

    # Export trained model parameters to C header
    export_c_header(final_model, "soundsort_model.h")


# Standard Python entry point
if __name__ == "__main__":
    main()