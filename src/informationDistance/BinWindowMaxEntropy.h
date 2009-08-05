#ifndef _IHA_WINDOW_ENTROPY_MAXIMIZER__H__
#define _IHA_WINDOW_ENTROPY_MAXIMIZER__H__
/*
 * Copyright (C) <2006> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Naeem Assif Mirza (University of Hertfordshire)
 * email:   n.a.mirza@herts.ac.uk
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include <list>

using namespace std;

/**
 * This class keeps a moving data window of sensor values
 * and uses it to estimate how the bin boundaries should be
 * arranged in order to maximize the entropy over the data window.
 * Adapted from Lars Olsson PhD thesis 2006 (Chapter 6)
 */
class BinWindowMaxEntropy {
private:

	int numBins;
	int windowSize;
	int histogramBins;

	// sumary information
	int *histogram;
	int numSamples;

	list<int> *dataWindow;

public:
	/**
	 * default Constructor
	 */
	BinWindowMaxEntropy():dataWindow(0) {
	}
	/**
	 * Constructor (initalizing)
	 * @param numbins - number of bins
	 * @param window - size of the datawindow
	 */
	BinWindowMaxEntropy(const unsigned char numbins, const int window, const int histbins) {
		numBins=numbins;
		windowSize=window;
		histogramBins=histbins;

		// Data will be stored in a moving window
		dataWindow = new list<int>;
		numSamples=0;

		// Summary data structure
		// holds the frequency of occurence of values in
		// the sensor reading
		histogram=new int[histogramBins];

		for (int i=0;i<histogramBins;i++) histogram[i]=0;
	}

	/**
	 * Destructor
	 */
	~BinWindowMaxEntropy() {
		delete dataWindow;
		delete [] histogram;
	}

	/**
	 * If the object is created by the default constructor, then
	 * this can be used to initalize the object
	 */
	void init(const unsigned char numbins, const int window, const int histbins) {
		if (dataWindow==0) {
			numBins=numbins;
			windowSize=window;
			histogramBins=histbins;

			// Data will be stored in a moving window
			dataWindow = new list<int>;
			numSamples=0;

			// Summary data structure
			// holds the frequency of occurence of values in
			// the sensor reading
			histogram=new int[histogramBins];

			for (int i=0;i<histogramBins;i++) histogram[i]=0;
		}
	}

	/**
	 * Add a new data line to the window and move
	 * the data window along.
	 */
	void putNextData(double x){
		int binned = getUniformBin(x,histogramBins);
		// add to our data window
		dataWindow->push_back(binned);

		// update the data structures
		histogram[binned]++;
		numSamples++;

		// move window along
		if (numSamples > windowSize) {
				// update the data structures
				int oldest_binned = dataWindow->front();

				histogram[oldest_binned]--;
				numSamples--;

				// remove the item
				dataWindow->pop_front();

		}
	}


	/**
	 * Get the bin for a given value (normalized 0..1)
	 * Adapted from Lars Olsson PhD thesis 2006 p111 (Chapter 6)
	 *
	 */
	int getBinMaxEntropy(double val) {
		int bin = _getBinMaxEntropy(val);
		if (bin>=numBins) bin=numBins-1;
		return bin;
	}

	int _getBinMaxEntropy(double val) {
		int valuesPerBin = windowSize / numBins;

		// use uniform binning where the window has not been half filled
		if (numSamples < windowSize/2) {
			return getUniformBin(val,numBins);
		}

		int sum=0; int total=0;
		int currBin=0;

		// keep a count of where we are in the histogram
		// in terms of the range of val values
		double currHistValue=0;
		double histInc=1.0/(double)histogramBins;

		for (int i=0;i<histogramBins;i++) {
			sum += histogram[i];
			total += histogram[i];
			if (total>windowSize) {
				fprintf(stderr,"ERROR: Too many samples\n");
			}
			currHistValue+=histInc;

			// check if we are in boundary *first*
			if (currHistValue >= val) {
				return currBin;
			}

			// slight change here - use >= to ensure top bin is used
			if (sum >= valuesPerBin) {
				sum = 0;
				currBin++;
			}
		}
		return currBin;

	}

	/**
	 * Function to calculate bin number
	 * for given (normalized) value
	 */
	int getUniformBin(double val, int nb) {
		int bin = (int) (val * (double) nb);
		if (bin>=nb) bin=(int) (nb-1);
		if (bin<=0) bin=0;
		return bin;
	}

	//----------------------------------------------------------------------------
	// some printing routines for testing

	/**
	 * print out the histogram
	 */
	void printHistogram() {
		fprintf(stderr,"Histogram: ");
		int total=0;
		for (int x=0;x<histogramBins;x++) {
			fprintf(stderr,"%d ",histogram[x]);
			total+=histogram[x];
		}
		fprintf(stderr," Total %d\n",total);
	}


};

#endif

