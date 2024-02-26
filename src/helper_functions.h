
class DoubleList {
private:
	double data[100];

public:
	int length;  // keeps track of current list length
	int max_length;

	DoubleList(int p_max_length) {
		max_length = p_max_length;
	}

	double sum() {
		double sum = 0;
		for (int i=0; i<length; i++) {
			sum += data[i];
		}

		return sum;
	}

	double average() {
		return sum() / length;
	}

	void append(double item) {
		// add item to end of list if list length is not too long
		if (length < max_length) data[length++] = item;
	}

	void remove(int index) {
		// remove item at index and shift array to left to fill gap
		if (index >= length) return;
		// create temporary array containing elements with index removed
		double temp[max_length - 1];
		// copy items before index
		for (int i = 0; i < index; i++) {
			temp[i] = data[i];
		}
		// copy items after index
		for (int i = index + 1; i < length; i++) {
			temp[i - 1] = data[i];
		}
		length--;

		// copy items back into data arr from temp arr
		for (int i = 0; i < length; i++) {
			data[i] = temp[i];
		}
	}

	void print() {
		String output = "[";

		for (int i = 0; i < length; i++) {
			output += String(data[i], 3);
			if (i < length - 1) {
				output += ", ";
			}
		}
		output += "]";

		Serial.println(output);
	}
};
