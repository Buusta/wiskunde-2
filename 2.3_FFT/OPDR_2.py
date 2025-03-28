import numpy as np
import matplotlib.pyplot as plt
import scipy.fft as fft
import scipy.io.wavfile as wav

# Load audio file
r, s = wav.read('sample.wav')
s = s[:, 0] # take Left channel

# FFT transform
freqs = fft.fftfreq(len(s), 1/r)
ffts = fft.fft(s)

# Low pass
cutoff_1 = 700
low_pass_mask = np.abs(freqs) < cutoff_1
ffts_low = ffts * low_pass_mask

# High pass
cutoff_2 = 2000
high_pass_mask = np.abs(freqs) > cutoff_2
ffts_high = ffts * high_pass_mask

# Band pass
cutoff_3 = 1500
cutoff_4 = 500
band_pass_mask = (np.abs(freqs) > cutoff_4) & (np.abs(freqs) < cutoff_3)
ffts_band = ffts * band_pass_mask

# IFFT to convert to audio
s_low = np.real(fft.ifft(ffts_low))
s_high = np.real(fft.ifft(ffts_high))
s_band = np.real(fft.ifft(ffts_band))

# Save audio files
wav.write('low_pass.wav', r, s_low.astype(np.int16))
wav.write('high_pass.wav', r, s_high.astype(np.int16))
wav.write('band_pass.wav', r, s_band.astype(np.int16))

# Stereo geluid werkend maken lukte niet :(