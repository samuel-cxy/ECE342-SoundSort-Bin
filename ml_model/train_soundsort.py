# pip install numpy scipy scikit-learn

import os
import numpy as np
from scipy.io import wavfile
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score

# =========================
# CONFIG
# =========================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_PATH = os.path.join(BASE_DIR, "my_dataset")
CLASSES = ["can", "glass", "paper", "plastic"]

FS = 16000
AUDIO_LEN = 8000
N_FFT = 1024
RANDOM_STATE = 42

HEADER_FILENAME = os.path.join(BASE_DIR, "soundsort_model.h")

print("Script folder:", BASE_DIR)
print("Dataset path:", DATASET_PATH)

# =========================
# WAV LOADING
# =========================
def load_wav_as_float(path):
    sr, x = wavfile.read(path)

    if sr != FS:
        raise ValueError(f"{path}: expected {FS} Hz, got {sr} Hz")

    # Convert stereo -> mono if needed
    if x.ndim > 1:
        x = x[:, 0]

    # Match STM32 normalization as closely as possible
    if x.dtype == np.int16:
        x = x.astype(np.float32) / 32768.0
    elif np.issubdtype(x.dtype, np.integer):
        info = np.iinfo(x.dtype)
        x = x.astype(np.float32) / max(abs(info.min), abs(info.max))
    else:
        x = x.astype(np.float32)

    # Force exactly 8000 samples
    if len(x) < AUDIO_LEN:
        x = np.pad(x, (0, AUDIO_LEN - len(x)))
    else:
        x = x[:AUDIO_LEN]

    return x


# =========================
# FEATURE EXTRACTION
# Matches patched STM32 code
# =========================
def extract_features_embedded(x):
    x = np.array(x, dtype=np.float32, copy=True)

    # ------------------------------------------
    # 0. Trim after last active sample
    # same as patched C
    # ------------------------------------------
    last_active = np.where(np.abs(x) > 0.03)[0]
    if last_active.size > 0:
        x[last_active[-1] + 1:] = 0.0

    # ------------------------------------------
    # 1. Time-domain features
    # ------------------------------------------
    abs_x = np.abs(x)
    sum_sq = np.sum(x * x)
    peak = np.max(abs_x) + 1e-12

    zcr_count = np.sum(
        ((x[1:] >= 0.0) & (x[:-1] < 0.0)) |
        ((x[1:] < 0.0) & (x[:-1] >= 0.0))
    )
    zcr = zcr_count / AUDIO_LEN

    third = AUDIO_LEN // 3
    e1 = np.sum(x[:third] * x[:third]) + 1e-12
    e3 = np.sum(x[-third:] * x[-third:]) + 1e-12
    decay = np.log(e1 / e3)

    active_ratio = np.sum(abs_x > 0.05) / AUDIO_LEN

    rms = np.sqrt(sum_sq / AUDIO_LEN + 1e-12)

    # ------------------------------------------
    # 2. Frequency-domain features
    # ------------------------------------------
    x_fft = x[:N_FFT].copy()

    # Same formula as C:
    # 0.5 * (1 - cos(2*pi*i/(N_FFT-1)))
    window = np.hanning(N_FFT).astype(np.float32)
    x_fft *= window

    S = np.abs(np.fft.rfft(x_fft)) ** 2
    total_energy = np.sum(S) + 1e-12

    bin_width = FS / N_FFT
    freqs = np.arange((N_FFT // 2) + 1, dtype=np.float32) * bin_width

    centroid = np.sum(freqs * S) / total_energy

    cumsum = np.cumsum(S)
    target = 0.85 * total_energy
    idx = int(np.searchsorted(cumsum, target))
    idx = min(idx, len(freqs) - 1)
    rolloff = freqs[idx]

    edges = [0, 300, 700, 1200, 2000, 3000, 4500, 6500, 8000]
    bands = []
    for i in range(len(edges) - 1):
        mask = (freqs >= edges[i]) & (freqs < edges[i + 1])
        band_e = np.sum(S[mask])
        bands.append(band_e / total_energy)

    feats = [rms, peak, zcr, decay, centroid, rolloff, active_ratio] + bands
    return np.array(feats, dtype=np.float32)


# =========================
# LOAD DATASET
# =========================
X = []
y = []

print("Extracting features from WAV files...")

for label, class_name in enumerate(CLASSES):
    class_dir = os.path.join(DATASET_PATH, class_name)

    if not os.path.isdir(class_dir):
        raise FileNotFoundError(f"Missing class folder: {class_dir}")

    wav_files = [f for f in os.listdir(class_dir) if f.lower().endswith(".wav")]
    print(f"{class_name}: {len(wav_files)} files")

    for file in wav_files:
        path = os.path.join(class_dir, file)
        audio = load_wav_as_float(path)
        feats = extract_features_embedded(audio)
        X.append(feats)
        y.append(label)

X = np.array(X, dtype=np.float32)
y = np.array(y, dtype=np.int32)

print(f"\nDataset shape: X={X.shape}, y={y.shape}")

# =========================
# TRAIN / TEST SPLIT
# =========================
X_train, X_test, y_train, y_test = train_test_split(
    X, y,
    test_size=0.2,
    stratify=y,
    random_state=RANDOM_STATE
)

print(f"Train: {len(X_train)}, Test: {len(X_test)}")

# =========================
# SCALE + TRAIN
# =========================
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

model = LogisticRegression(
    max_iter=2000,
    solver="lbfgs",
    random_state=RANDOM_STATE
)

model.fit(X_train_scaled, y_train)

# =========================
# EVALUATION
# =========================
y_pred = model.predict(X_test_scaled)
acc = accuracy_score(y_test, y_pred)

print(f"\nTest accuracy: {acc:.4f}\n")
print("Classification report:")
print(classification_report(y_test, y_pred, target_names=CLASSES))

print("Confusion matrix:")
print(confusion_matrix(y_test, y_pred))

# =========================
# EXPORT TO HEADER
# =========================
def export_to_header(model, scaler, classes, filename=HEADER_FILENAME):
    num_classes = len(classes)
    num_features = model.coef_.shape[1]

    with open(filename, "w") as f:
        f.write("#ifndef SOUNDSORT_MODEL_H\n")
        f.write("#define SOUNDSORT_MODEL_H\n\n")

        f.write(f"#define NUM_CLASSES {num_classes}\n")
        f.write(f"#define NUM_FEATURES {num_features}\n\n")

        f.write("static const char* CLASS_NAMES[] = {")
        f.write(", ".join([f'"{c}"' for c in classes]))
        f.write("};\n\n")

        f.write("static const float FEATURE_MEAN[NUM_FEATURES] = {")
        f.write(", ".join([f"{m:.9g}f" for m in scaler.mean_]))
        f.write("};\n")

        f.write("static const float FEATURE_SCALE[NUM_FEATURES] = {")
        f.write(", ".join([f"{s:.9g}f" for s in scaler.scale_]))
        f.write("};\n\n")

        f.write("static const float LR_WEIGHTS[NUM_CLASSES][NUM_FEATURES] = {\n")
        for row in model.coef_:
            f.write("    {")
            f.write(", ".join([f"{w:.9g}f" for w in row]))
            f.write("},\n")
        f.write("};\n\n")

        f.write("static const float LR_BIAS[NUM_CLASSES] = {")
        f.write(", ".join([f"{b:.9g}f" for b in model.intercept_]))
        f.write("};\n\n")

        f.write("#endif\n")

export_to_header(model, scaler, CLASSES, HEADER_FILENAME)
print(f"\nExported model header to: {HEADER_FILENAME}")