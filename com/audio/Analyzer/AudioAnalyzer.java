package com.audio.Analyzer;

import org.jtransforms.fft.DoubleFFT_1D;

import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import java.io.File;
import java.util.*;


public class AudioAnalyzer {

    private double[] frequencyAxis;
    private double bassAverage;
    private final double bassUpperLimit = 300;
    private final double bassLowerLimit = 80;
    private double sampleRate = 8000;

    /**
     * Read audio file and store data in audioDoubleData
     * @param filename Name of audio file
     * @return Raw audio data in double format and duration of audio in seconds
     */
    public AudioData readFile(String filename) {
        File inputFile = new File(filename);

        try {
            // Create AudioInputStream object
            AudioInputStream audioInputStream = AudioSystem.getAudioInputStream(inputFile);
            int numFrames = (int) audioInputStream.getFrameLength();
            int frameSize = audioInputStream.getFormat().getFrameSize();
            int numChannels = audioInputStream.getFormat().getChannels();
            sampleRate = audioInputStream.getFormat().getSampleRate();

            // Convert audio bytes to double values
            double[] audioDoubleData = new double[numFrames * numChannels];

            byte[] audioBytes = new byte[numFrames * frameSize];
            int bytesRead = audioInputStream.read(audioBytes);

            if (bytesRead != -1) {
                // Convert the bytes to double values
                int byteIndex = 0;
                for (int i = 0; i < numFrames; i++) {
                    for (int channel = 0; channel < numChannels; channel++) {
                        // Convert two bytes to a 16-bit signed value in the range [-1.0, 1.0]
                        int sample = (audioBytes[byteIndex++] & 0xFF) | (audioBytes[byteIndex++] << 8);
                        audioDoubleData[i * numChannels + channel] = (double) sample / 32768.0;
                    }
                }
            }

            // calculate time duration
            double durationInSeconds = (double) numFrames / audioInputStream.getFormat().getFrameRate();
            int sampleRate = (int) audioInputStream.getFormat().getSampleRate();

            // Create AudioData object
            AudioData audioDataObject = new AudioData();
            audioDataObject.data = audioDoubleData;
            audioDataObject.duration = durationInSeconds;
            audioDataObject.sampleRate = sampleRate;
            return audioDataObject;
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        return null;
    }

    /**
     * Extract Magnitude from FFT result
     * @param fftResult Result of fft
     * @return Magnitude data in double format
     * @note Taken from <a href="https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/431776b51fb396e3085c3de47ff4839ba7124150/ihmc-avatar-interfaces/src/main/java/us/ihmc/avatar/sensors/microphone/SoundDetector.java#L414">...</a>
     */
    private static double[] extractMagnitude(double[] fftResult)
    {
        int n = fftResult.length;
        double[] magnitude = new double[n / 2];

        for (int k = 0; k < n / 2; k++)
        {
            double real = fftResult[2 * k];
            double imag = fftResult[2 * k + 1];

            if (k == 0)
                imag = 0; // This is due to storing a[1] = Re[n/2] when n is even and = Im[n/2-1] when n is odd, as per the fft specs. Just ignore that term for now...

            double mag = Math.sqrt(real * real + imag * imag);
            magnitude[k] = mag;
        }

        return magnitude;
    }

    /**
     * Extract Phase from FFT result
     * @param audioData Result of fft
     * @return Phase data in double format
     * @note Taken from
     */

    public double[] performFFT(AudioData audioData) {
        int fftlength = audioData.data.length;
        double[] fftInput = new double[fftlength];
        System.arraycopy(audioData.data, 0, fftInput, 0, fftlength);

        DoubleFFT_1D fft = new DoubleFFT_1D(fftlength);
        fft.realForward(fftInput);

        return extractMagnitude(fftInput);
    }

    public double[] convertToLogScale(double[] magnitude) {
        double[] logScale = new double[magnitude.length];
        double reference = 0.00002;

        for (int i = 0; i < magnitude.length; i++)
            logScale[i] = 20 * Math.log10(magnitude[i] / reference);

        return logScale;
    }


    public double[][] average(double[] magnitudeData, int sampleRate, double octave) {
        double[] frequencyBins = generateFrequencyBins(octave, sampleRate);
        double[] average = new double[frequencyBins.length - 1];

        int startIdx = 0;
        int endIdx;
        // Calculate the frequency band averages
        for (int i = 0; i < frequencyBins.length - 1; i++) {
            endIdx = findFrequencyIndex(frequencyBins[i + 1], sampleRate, magnitudeData.length) + 1;
            average[i] = calculateMeanMagnitude(magnitudeData, startIdx, endIdx);
            startIdx = endIdx;
        }
        average = convertToLogScale(average);

        return new double[][]{frequencyBins, average};
    }

    public double[][] getPeakValues(double[][] freq_spl) {
        List<Map.Entry<Double, Double>> peakValues = new ArrayList<>();

        for (int i = 0; i < freq_spl[0].length; i++) {
            if (freq_spl[0][i] > bassLowerLimit && freq_spl[0][i] < bassUpperLimit) {
                peakValues.add(new AbstractMap.SimpleEntry<>(freq_spl[0][i], freq_spl[1][i]));
            }
        }

        getBassAverage(freq_spl);

        // Apply room correction filter
        // spl = spl + 2.6 + 0.0026 * frequency if frequency < 1000 Hz
        // spl = spl - 0.1 - 0.001 * frequency if frequency > 1000 Hz
        peakValues = applyRoomCorrectionFilter(peakValues);

        // Sort the entryList based on the values using a custom comparator
        peakValues.sort(Map.Entry.comparingByValue());

        // Get the last 5 entries
        peakValues = peakValues.subList(peakValues.size() - 5, peakValues.size());

        double[][] peakValuesArray = new double[2][peakValues.size()];
        for (int i = 0; i < peakValues.size(); i++) {
            peakValuesArray[0][i] = peakValues.get(i).getKey();
            peakValuesArray[1][i] = peakValues.get(i).getValue();
        }

        return peakValuesArray;
    }

    private static List<Map.Entry<Double, Double>> applyRoomCorrectionFilter(List<Map.Entry<Double, Double>> peakValues) {
        for (int i = 0; i < peakValues.size(); i++) {
            double frequency = peakValues.get(i).getKey();
            double spl = peakValues.get(i).getValue();

            if (frequency < 1000) {
                spl = spl + 2.6 + 0.0026 * frequency;
            } else {
                spl = spl - 0.1 - 0.001 * frequency;
            }

            peakValues.set(i, new AbstractMap.SimpleEntry<>(frequency, spl));
        }

        return peakValues;
    }

    private void getBassAverage(double[][] freq_spl) {
        // Bass Average
        double bassSum = 0;
        int bassCount = 0;
        for (int i = 0; i < freq_spl[0].length; i++) {
            if (freq_spl[0][i] > 20 && freq_spl[0][i] < 70) {
                bassSum += freq_spl[1][i];
                bassCount++;
            }
        }
        bassAverage = bassSum / bassCount;
    }


    private double[] generateFrequencyBins(double octave, int sampleRate) {
        LinkedList<Double> frequency_bins = new LinkedList<>();
        double half_sample_rate = (double) sampleRate / 2;
        double current_frequency = 20;

        while (current_frequency < half_sample_rate) {
            frequency_bins.add(current_frequency);
            current_frequency = current_frequency * Math.pow(2, 1.0 / octave);
        }
        frequency_bins.add(half_sample_rate);

        double[] frequency_bins_array = new double[frequency_bins.size()];
        for (int i = 0; i < frequency_bins.size(); i++) {
            frequency_bins_array[i] = frequency_bins.get(i);
        }
        return frequency_bins_array;
    }


    public int findFrequencyIndex(double targetFrequency, int sampleRate, int dataSize) {
        double startingFrequency = 20.0; // Starting frequency in Hz
        double nyquistFrequency = sampleRate / 2.0;
        double frequencyIncrement = (nyquistFrequency - startingFrequency) / dataSize;
        return (int) ((targetFrequency - startingFrequency) / nyquistFrequency * dataSize);
    }

    public double[] generateFrequencyAxis(int sampleRate, int dataSize) {
        frequencyAxis = new double[dataSize];
        double nyquistFrequency = sampleRate / 2.0;
        double frequencyIncrement = nyquistFrequency / dataSize;

        for (int i = 0; i < dataSize; i++) {
            frequencyAxis[i] = i * frequencyIncrement;
        }

        return frequencyAxis;
    }

    private double calculateMeanMagnitude(double[] magnitudeData, int startIdx, int endIdx) {
        double sum = 0;
        for (int i = startIdx; i < endIdx; i++) sum += magnitudeData[i];
        return sum / (endIdx - startIdx);
    }

    public double[] getLowHighFrequencies(double[][] spl, double[] frequencyAxis) {
        double peakFrequency = spl[0][spl[0].length - 1];
        double min = 0, max = 0;
        double min_= 0, max_= 0;

        for (double frequency : frequencyAxis) {
            if (frequency < peakFrequency) {
                min = frequency;
            } else if (frequency > peakFrequency) {
                max = frequency;
            }

            if (frequency > bassUpperLimit) {
                break;
            }
        }

        // Find the closest frequency to the peak frequency
        if (peakFrequency - min < max - peakFrequency) {
            min_ = min;
        } else {
            min_ = max;
        }

        // Find the other frequency using geometric mean
        max_ = peakFrequency* peakFrequency / min_;
        return new double[]{min_, max_};
    }

    public double[] calculateFilter(double[] peakFrequencies, double[][] peaks) {
        double[] filterValues = new double[3];
        double min = peakFrequencies[0];
        double max = peakFrequencies[1];
        double peakFrequency = peaks[0][peaks[0].length - 1];
        double peakDB = peaks[1][peaks[1].length - 1];

        double Q = peakFrequency / Math.abs(max - min);
        double gain = bassAverage - peakDB;
        if (gain < -5) gain = -5;
        if (Q > 10) Q = 10;

        filterValues[0] = peakFrequency;
        filterValues[1] = Q;
        filterValues[2] = gain;

        return filterValues;
    }
}