import numpy as np
import wave
import struct

def generate_sine_wave(frequency, duration, sample_rate=44100, amplitude=32767):
    """Generate a sine wave with the specified frequency and duration."""
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    sine_wave = np.sin(2 * np.pi * frequency * t) * amplitude
    return sine_wave.astype(np.int16)

def save_wave_file(filename, audio_data, sample_rate=44100):
    """Save the given audio data as a WAV file."""
    with wave.open(filename, 'w') as wav_file:
        n_channels = 1  # Mono
        sampwidth = 2  # 2 bytes per sample (16 bits)
        n_frames = len(audio_data)
        comptype = "NONE"
        compname = "not compressed"
        wav_file.setparams((n_channels, sampwidth, sample_rate, n_frames, comptype, compname))

        for sample in audio_data:
            wav_file.writeframes(struct.pack('<h', sample))

def generate_bomb_drop_sound(duration=3, sample_rate=44100):
    """Generate a sound like a whistling bomb dropping with decreasing frequency."""
    start_frequency = 1000  # Start high-pitched
    end_frequency = 100      # End low-pitched
    frequencies = np.linspace(start_frequency, end_frequency, int(sample_rate * duration))
    bomb_sound = np.array([])

    for frequency in frequencies:
        wave_part = generate_sine_wave(frequency, 1/sample_rate, sample_rate)
        bomb_sound = np.concatenate((bomb_sound, wave_part))
    
    return bomb_sound.astype(np.int16)

def generate_laser_shot_sound(duration=0.5, sample_rate=44100):
    """Generate a high-pitched laser shot sound."""
    frequency = 800  # Base frequency of the laser
    modulating_frequency = 50  # The frequency that modulates the laser sound
    
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    laser_wave = (np.sin(2 * np.pi * frequency * t) + np.sin(2 * np.pi * modulating_frequency * t)) * 16384
    return laser_wave.astype(np.int16)

def create_empty_wav(filename, duration=1.0, sample_rate=44100):
    # Create an empty WAV file
    n_samples = int(sample_rate * duration)
    
    with wave.open(filename, 'w') as wav_file:
        wav_file.setnchannels(1)  # Mono
        wav_file.setsampwidth(2)  # 2 bytes per sample
        wav_file.setframerate(sample_rate)
        
        # Write silent samples
        for _ in range(n_samples):
            wav_file.writeframes(struct.pack('h', 0))

def main():
    # Create whistle.wav
    create_empty_wav('whistle.wav')
    print("Created whistle.wav")

    # Create laser.wav
    create_empty_wav('laser.wav')
    print("Created laser.wav")

    print("WAV files created. Exiting program.")

if __name__ == "__main__":
    main()
