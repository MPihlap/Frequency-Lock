import math
import sounddevice as sd

def goertzel(samples, sample_rate, *freqs):
    """
    Implementation of the Goertzel algorithm, useful for calculating individual
    terms of a discrete Fourier transform.

    `samples` is a windowed one-dimensional signal originally sampled at `sample_rate`.

    The function returns 2 arrays, one containing the actual frequencies calculated,
    the second the coefficients `(real part, imag part, power)` for each of those frequencies.
    For simple spectral analysis, the power is usually enough.

    Example of usage :
        
        freqs, results = goertzel(some_samples, 44100, (400, 500), (1000, 1100))
    """
    window_size = len(samples)
    f_step = sample_rate / float(window_size)
    f_step_normalized = 1.0 / window_size

    # Calculate all the DFT bins we have to compute to include frequencies
    # in `freqs`.
    bins = set()
    for f_range in freqs:
        f_start, f_end = f_range
        k_start = int(math.floor(f_start / f_step))
        k_end = int(math.ceil(f_end / f_step))

        if k_end > window_size - 1: raise ValueError('frequency out of range %s' % k_end)
        bins = bins.union(range(k_start, k_end))

    # For all the bins, calculate the DFT term
    n_range = range(0, window_size)
    freqs = []
    results = []
    for k in bins:

        # Bin frequency and coefficients for the computation
        f = k * f_step_normalized
        w_real = 2.0 * math.cos(2.0 * math.pi * f)
        w_imag = math.sin(2.0 * math.pi * f)

        # Doing the calculation on the whole sample
        d1, d2 = 0.0, 0.0
        for n in n_range:
            y  = samples[n] + w_real * d1 - d2
            d2, d1 = d1, y

        # Storing results `(real part, imag part, power)`
        results.append((
            0.5 * w_real * d1 - d2, w_imag * d1,
            d2**2 + d1**2 - w_real * d1 * d2)
        )
        freqs.append(f * sample_rate)
    return freqs, results

def max_freq(freqs, results):
    max_mag = -1
    m_index = -1
    for i in range(len(freqs)):
        magnitude = results[i][2][0]
        if magnitude > 50:
            if magnitude > max_mag:
                max_mag = magnitude
                m_index = i
    return m_index
                

root = 220 # Hz
# Progression - 1, low 5, major 3, 1 so 1:1, 3:4, 5:4, 1:1
progression = [root, root * 3/4, root * 5/4, root]
progression_counter = 0
duration = 1
fs = 2000
sd.default.samplerate = fs

while True:
    target_freq = progression[progression_counter]
    rec = sd.rec(int(duration)*fs, channels=1, blocking=True)
    freqs, results = goertzel(rec, fs, (target_freq - 2, target_freq + 2))
    max_index = max_freq(freqs, results)
    if max_index > 0:
        if progression_counter < 3:
            print("Stage "+str(progression_counter) + " complete.")
            print(freqs[max_index])
            progression_counter += 1
        else:
            print("Lock open!")
            break
    else:
        print(max_index)
