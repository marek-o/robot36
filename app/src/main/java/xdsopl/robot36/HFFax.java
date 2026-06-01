/*
HF Fax mode

Copyright 2025 Marek Ossowski <marek0ossowski@gmail.com>
*/

package xdsopl.robot36;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Rect;

/**
 * HF Fax, IOC 576, 120 lines per minute
 */
public class HFFax extends BaseMode {
	private final ExponentialMovingAverage lowPassFilter;
	private final String name;
	private final int sampleRate;
	private final float[] cumulated;
	private int horizontalShift = 0;
	private boolean shouldSaveImageFlag = false;

	private int lastStoreLines = 0;
	private int lastStartLines = Integer.MIN_VALUE;
	private int totalLines = 0;

	HFFax(int sampleRate) {
		this.name = "HF Fax";
		lowPassFilter = new ExponentialMovingAverage();
		this.sampleRate = sampleRate;
		cumulated = new float[getWidth()];
	}

	private float freqToLevel(float frequency, float offset) {
		return 0.5f * (frequency - offset + 1.f);
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public int getVISCode() {
		return -1;
	}

	@Override
	public int getWidth() {
		return 1808;
	}

	@Override
	public int getHeight() {
		return 1200;
	}

	@Override
	public int getFirstPixelSampleIndex() {
		return 0;
	}

	@Override
	public int getFirstSyncPulseIndex() {
		return -1;
	}

	@Override
	public int getScanLineSamples() {
		return sampleRate / 2;
	}

	@Override
	public void resetState() {
	}

	@Override
	public Bitmap postProcessScopeImage(Bitmap bmp) {
		int realWidth = 1808;
		int realHorizontalShift = horizontalShift * realWidth / getWidth();
		Bitmap bmpMutable = Bitmap.createBitmap(realWidth, bmp.getHeight(), Bitmap.Config.ARGB_8888);
		Canvas canvas = new Canvas(bmpMutable);
		if (horizontalShift > 0) {
			canvas.drawBitmap(
					bmp,
					new Rect(0, 0, horizontalShift, bmp.getHeight()),
					new Rect(realWidth - realHorizontalShift, 0, realWidth, bmp.getHeight()),
					null);
		}
		canvas.drawBitmap(
				bmp,
				new Rect(horizontalShift, 0, getWidth(), bmp.getHeight()),
				new Rect(0, 1, realWidth - realHorizontalShift, bmp.getHeight() + 1),
				null);

		return bmpMutable;
	}

	@Override
	public boolean decodeScanLine(PixelBuffer pixelBuffer, float[] scratchBuffer, float[] scanLineBuffer, int scopeBufferWidth, int syncPulseIndex, int scanLineSamples, float frequencyOffset) {
		if (syncPulseIndex < 0 || syncPulseIndex + scanLineSamples > scanLineBuffer.length)
			return false;
		int horizontalPixels = getWidth();
		lowPassFilter.cutoff(horizontalPixels, 2 * scanLineSamples, 2);
		lowPassFilter.reset();
		for (int i = 0; i < scanLineSamples; ++i)
			scratchBuffer[i] = lowPassFilter.avg(scanLineBuffer[i]);
		lowPassFilter.reset();
		for (int i = scanLineSamples - 1; i >= 0; --i)
			scratchBuffer[i] = freqToLevel(lowPassFilter.avg(scratchBuffer[i]), frequencyOffset);

		float[] grays = new float[horizontalPixels];

		for (int i = 0; i < horizontalPixels; ++i) {
			int position = (i * scanLineSamples) / horizontalPixels;
			int color = ColorConverter.GRAY(scratchBuffer[position]);
			float gray = Color.luminance(color);
			pixelBuffer.pixels[i] = color;
			grays[i] = gray;

			//accumulate recent values, forget old
			float decay = 0.99f;
			cumulated[i] = cumulated[i] * decay + gray * (1 - decay);
		}

		totalLines++;

		boolean shouldSave = false;

		final float threshold = 0.08f;

		//start/stop tone length: 5 seconds -> 10 lines
		//start tone: 300Hz -> 150 per line
		if (DFT(grays, 150).abs() / getWidth() > threshold) {
			lastStartLines = totalLines;
		}

		//stop tone: 450Hz -> 225 per line
		if (DFT(grays, 225).abs() / getWidth() > threshold) {
			shouldSave = true;

			//forget about last start, no need to save again
			lastStartLines = Integer.MIN_VALUE;
		}

		//save if one screen after start and no marker found (some images are larger than one screen so it can happen even with strong signal)
		//1280 is a bit too small for 11 minute images, but there is a margin at the start which can be skipped
		if (totalLines - lastStartLines == 1280 + 128) shouldSave = true;

		//save if two screens after start and no marker
		if (totalLines - lastStartLines == 1280 + 128 + 1280) shouldSave = true;

		if (shouldSave && (totalLines - lastStoreLines) > 20) {
			shouldSaveImageFlag = true;
			lastStoreLines = totalLines;
		}

		//try to detect "sync": thick white margin
		int bestIndex = 0;
		float bestValue = 0;
		for (int x = 0; x < getWidth(); ++x)
		{
			float val = cumulated[x];
			if (val > bestValue)
			{
				bestIndex = x;
				bestValue = val;
			}
		}

		horizontalShift = bestIndex;

		pixelBuffer.width = horizontalPixels;
		pixelBuffer.height = 1;
		return true;
	}

	public boolean shouldSaveImage() {
		if (shouldSaveImageFlag) {
			shouldSaveImageFlag = false;
			return true;
		}

		return false;
	}

	private Complex DFT(float[] input, int bin) {
		Complex result = new Complex();
		for (int i = 0; i < input.length; ++i) {
			float oscR = (float)Math.sin(i * 2 * Math.PI * bin / input.length);
			float oscI = (float)Math.cos(i * 2 * Math.PI * bin / input.length);
			result = result.add(new Complex(oscR * input[i], oscI * input[i]));
		}
		return result;
	}
}
