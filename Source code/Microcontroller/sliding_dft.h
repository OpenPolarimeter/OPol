/**
Sliding discrete Fourier transform (C++)

MIT License
----

Copyright (c) 2016 Bronson Philippa

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#define _USE_MATH_DEFINES
#include <complex>
#include <math.h>

template <class NumberFormat, size_t DFT_Length>
class SlidingDFT
{
private:
	/// Are the frequency domain values valid? (i.e. have at elast DFT_Length data
	/// points been seen?)
	bool data_valid = false;
  
	/// Time domain samples are stored in this circular buffer.
	NumberFormat x[DFT_Length] = { 0 };

	/// Index of the next item in the buffer to be used. Equivalently, the number
	/// of samples that have been seen so far modulo DFT_Length.
	size_t x_index = 0;

	/// Twiddle factors for the update algorithm
	std::complex<NumberFormat> twiddle[DFT_Length];

	/// Frequency domain values (unwindowed!)
	std::complex<NumberFormat> S[DFT_Length];

public:
	/// Frequency domain values (windowed)
	std::complex<NumberFormat> dft[DFT_Length];

	/// A damping factor introduced into the recursive DFT algorithm to guarantee
	/// stability.
	NumberFormat damping_factor = std::nexttoward((long double)1, (long double)0);

	/// Constructor
	SlidingDFT()
	{
		const std::complex<NumberFormat> j(0.0, 1.0);
		const NumberFormat N = DFT_Length;

		// Compute the twiddle factors, and zero the x and S arrays
		for (size_t k = 0; k < DFT_Length; k++) {
			NumberFormat factor = (NumberFormat)(2.0 * M_PI) * k / N;
			this->twiddle[k] = std::exp(j * factor);
			this->S[k] = 0;
			this->x[k] = 0;
		}
	}

	/// Determine whether the output data is valid
	bool is_data_valid()
	{
		return this->data_valid;
	}

	/// Update the calculation with a new sample
	/// Returns true if the data are valid (because enough samples have been
	/// presented), or false if the data are invalid.
	bool update(NumberFormat new_x)
	{
		// Update the storage of the time domain values
		const NumberFormat old_x = this->x[this->x_index];
		this->x[this->x_index] = new_x;

		// Update the DFT (only first few bins)
		const NumberFormat r = this->damping_factor;
		const NumberFormat r_to_N = pow(r, (NumberFormat)DFT_Length);
		for (size_t k = 0; k < 400; k++) {
			this->S[k] = this->twiddle[k] * (r * this->S[k] - r_to_N * old_x + new_x);
		}

		// Apply the Hanning window
		//this->dft[0] = (NumberFormat)0.5*this->S[0] - (NumberFormat)0.25*(this->S[DFT_Length - 1] + this->S[1]);
		for (size_t k = 1; k < 399; k++) {
			this->dft[k] = (NumberFormat)0.5*this->S[k] - (NumberFormat)0.25*(this->S[k - 1] + this->S[k + 1]);
		}
		//this->dft[DFT_Length - 1] = (NumberFormat)0.5*this->S[DFT_Length - 1] - (NumberFormat)0.25*(this->S[DFT_Length - 2] + this->S[0]);

		// Increment the counter
		this->x_index++;
		if (this->x_index >= DFT_Length) {
			this->data_valid = true;
			this->x_index = 0;
		}

		// Done.
		return this->data_valid;
	}

  //Update the calculation with new sample but only around values of peak in question
  

};
