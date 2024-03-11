#pragma once

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
		for (int i = 0; i < length; i++) {
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



class ImuComponent {
private:
	int num_avg;
	DoubleList x_list = DoubleList(5);
	DoubleList y_list = DoubleList(5);
	DoubleList z_list = DoubleList(5);

	// if new value is > 10x the current average, then do not save value

	bool is_outliar(float old_avg, float new_val) {
		if (old_avg == 0 || new_val == 0) {
			return false;
		} else {
			float factor = 1000.0;
			if (abs(new_val) < 1) {
				factor = 100;
			} else if (abs(new_val) < 5) {
				factor = 500;
			} else if (abs(new_val) < 10) {
				factor = 1000;
			} else if (abs(new_val) < 100) {
				factor = 4000;
			} else {
				factor = 4000;
			}

			factor *= outliar_multiplier;

			if ((new_val > (old_avg + factor) || new_val < (old_avg - factor))) {
				return true;
			}
			return false;
		}
	}

	void add_new_x(float new_x) {
		if (is_outliar(x, new_x)) {
			return;
		}

		if (x_list.length < x_list.max_length) {
			x_list.append(new_x);
		} else {
			x_list.remove(0);
			x_list.append(new_x);
		}

		x = x_list.average();
	}

	void add_new_y(float new_y) {
		if (is_outliar(y, new_y)) {
			return;
		}

		if (y_list.length < y_list.max_length) {
			y_list.append(new_y);
		} else {
			y_list.remove(0);
			y_list.append(new_y);
		}

		y = y_list.average();
	}

	void add_new_z(float new_z) {
		if (is_outliar(z, new_z)) {
			return;
		}

		if (z_list.length < z_list.max_length) {
			z_list.append(new_z);
		} else {
			z_list.remove(0);
			z_list.append(new_z);
		}

		z = z_list.average();
	}

public:
	float x;
	float y;
	float z;

	float outliar_multiplier = 1;

	ImuComponent(int pnum_avg, float p_outliar_multiplier) {
		x = 0;
		y = 0;
		z = 0;

		x_list.max_length = pnum_avg;
		y_list.max_length = pnum_avg;
		z_list.max_length = pnum_avg;

		outliar_multiplier = p_outliar_multiplier;
	}

	// create method that takes in current values of x, y, and z
	// and adds them to the list
	// finds average of list and assigns to x, y, and z
	void add_new_values(float new_x, float new_y, float new_z) {
		add_new_x(new_x);
		add_new_y(new_y);
		add_new_z(new_z);
	}





};
