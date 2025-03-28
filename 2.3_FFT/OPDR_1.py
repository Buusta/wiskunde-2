import csv
import numpy as np
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

# Laad de data
data = []

with open("data.csv", newline="", encoding="utf-8") as file:
    reader = csv.reader(file)
    data = [float(row[0]) for row in reader]  # Zorg dat data float is

# Bemonsteringsfrequentie 1 GHz
sampling_rate = 1e9

# FFT van de data
fxt = fft(data)

# Positieve frequenties
freqx = fftfreq(len(data), 1 / sampling_rate)
positive_freqs = freqx[:len(fxt)//2]
positive_fxt = np.abs(fxt[:len(fxt)//2])

# Zoek pieken
peaks, _ = find_peaks(positive_fxt)

# Plot de resultaten
plt.plot(positive_freqs, positive_fxt)
plt.plot(positive_freqs[peaks], positive_fxt[peaks], "x", color='red')  # Piekmarkeringen
plt.title("FFT met pieken")
plt.xlabel("Frequentie (Hz)")
plt.ylabel("Amplitude")
plt.grid(True)
plt.show()

# Inzoomen op de pieken en plot per piek
for peak in peaks:
    # Bepaal het bereik rond de piek
    peak_freq = positive_freqs[peak]
    range_min = peak_freq - 1e7  # 10 MHz lager
    range_max = peak_freq + 1e7  # 10 MHz hoger

    # Filter de frequenties en amplitudes binnen dit bereik
    zoomed_freqs = positive_freqs[(positive_freqs >= range_min) & (positive_freqs <= range_max)]
    zoomed_fxt = positive_fxt[(positive_freqs >= range_min) & (positive_freqs <= range_max)]

    # Plot de gefilterde frequentie en amplitude voor de piek
    plt.plot(zoomed_freqs, zoomed_fxt)
    plt.title(f"Zoom op piek rond {peak_freq/1e6:.2f} MHz")
    plt.xlabel("Frequentie (Hz)")
    plt.ylabel("Amplitude")
    plt.grid(True)
    plt.show()

# Piekfrequenties
print("Piekfrequenties:", positive_freqs[peaks])
