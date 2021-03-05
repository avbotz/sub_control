"""
Launch a GUI to edit and tune the sub's PID gains.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from datetime import datetime
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from sub_control_interfaces.srv import ControlWrite

# Don't forget to update these gains after the gains are updated in Nautical
GAINS = [
	[ 2.00, 0.00, 2.50 ],
	[ 2.50, 0.00, 2.00 ],
	[ 1.75, 0.00, 0.00 ],
	[ 0.10, 0.00, 0.05 ],
	[ 0.10, 0.00, 0.05 ],
	[ 0.10, 0.00, 0.05 ],
	[ 0.85, 0.00, 0.10 ]
]

DOF = 6
NUM_GAINS = 3

class TuningGui(QMainWindow):

	def __init__(self):
		QMainWindow.__init__(self)

		self.setMinimumSize(QSize(1200, 400))    
		self.setWindowTitle("PID Tuner") 

		# Create labels
		self.createLabel("P", 125, 43)
		self.createLabel("I", 225, 43)
		self.createLabel("D", 325, 43)
		self.createLabel("F", 40, 75)
		self.createLabel("H", 40, 110)
		self.createLabel("V", 40, 145)
		self.createLabel("Y", 40, 180)
		self.createLabel("P", 40, 215)
		self.createLabel("R", 40, 250)
		self.createLabel("A", 40, 285)
		self.createLabel("P = proportional gain (how aggressively motors run)", 450, 38)
		self.createLabel("I = integral gain (keep this at 0)", 450, 68)
		self.createLabel("D = derivative gain (damps movement, helps prevent overshoot)", 450, 98)
		self.createLabel("F = front degree of freedom", 450, 128)
		self.createLabel("H = horizontal degree of freedom", 450, 158)
		self.createLabel("V = vertical degree of freedom", 450, 188)
		self.createLabel("Y = yaw degree of freedom", 450, 218)
		self.createLabel("P = pitch degree of freedom (generally don't touch this)", 450, 248)
		self.createLabel("R = roll degree of freedom (generally don't touch this)", 450, 278)
		self.createLabel("A = altitude degree of freedom", 450, 308)
		self.createLabel("Gains are saved in BB's 'gains' folder on each update", 450, 338)

		# Create 7 rows of input boxes for each degree of freedom, 3 columns for each type of gain
		self.input_boxes = self.createBoxes(80, 70)
		self.initInputBoxes()

		# Create button that will send the updated gains to Nautical
		pybutton = QPushButton("Update", self)
		pybutton.clicked.connect(self.clickMethod)
		pybutton.resize(200, 32)
		pybutton.move(125, 325)      

		# Setup service client to send commands to sub_control's service
		self.node = Node("temp")
		self.client = self.node.create_client(ControlWrite, 'control_write')
		self.request = ControlWrite.Request()

	def createBoxes(self, x, y):
		""" Creates 7 rows of 3 boxes each to type PID values into """
		original_x = x
		boxes = []
		box_width = 100
		box_height = 35
		for i in range(DOF + 1):
			for j in range(NUM_GAINS):
				box = QLineEdit(self)
				box.move(x, y)
				box.resize(box_width, box_height)
				x += box_width
				boxes.append(box)
			x = original_x
			y += box_height
		return boxes

	def createLabel(self, text, x, y):
		label = QLabel(self)
		label.resize(800, 25)
		label.setText(text)
		label.setFont(QFont("Consolas"))
		label.move(x, y)

	def initInputBoxes(self):
		""" Place the default gain values into the input boxes """
		for i in range(DOF + 1):
			for j in range(NUM_GAINS):
				overall_index = (i * 3) + j
				self.input_boxes[overall_index].setText(str(GAINS[i][j]))

	def isValidInputBoxes(self):
		""" See if every input box has a valid number """
		for box in self.input_boxes:
			read = str(box.text())
			try:
				float(read)
			except ValueError:
				return False
			if "+" in read:
				return False
		return True

	def readInputBoxes(self):
		""" Read the inputted gains into a list """
		gains = []
		for i in range(DOF + 1):
			row = []
			for j in range(NUM_GAINS):
				overall_index = (i * 3) + j
				read = float(self.input_boxes[overall_index].text())
				row.append(read)
			gains.append(row)
		return gains

	def parseGain(self, gain):
		""" 
		Takes in string gain, returns the same gain as a 4 character string to look nice 
		when writing to file.
		"""
		num_chars = len(gain)
		if num_chars < 4:
			if "." in gain:
				for i in range(4 - num_chars):
					gain += "0"
			else:
				gain += "."
				num_chars += 1
				for i in range(4 - num_chars):
					gain += "0"
		return gain

	def saveGains(self, gains):
		# Format the gains nicely into a string
		string = "{\n"
		for i in range(DOF + 1):
			string += "\t{ "
			for j in range(NUM_GAINS):
				gain = str(gains[i][j])
				gain = self.parseGain(gain)
				string += gain
				if j != 2:
					string += ","
				string += " "
			if i != DOF:
				string += "},\n"
			else:
				string += "}\n"
		string += "}"

		# Save gains to .txt file in sub interface's log folder
		filepath = ("gains/" + datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + ".txt")
		with open(filepath, "w+") as file:
			file.write("IMPORTANT: Don't forget to update the new gains in Nautical's "
				"config.hpp and the constant GAINS in sub_control's "
				"tuning.py!\n\nHere are the new gains\n"+ string)

	def clickMethod(self):
		""" Once update button is clicked, sends gains to Nautical """
		if self.isValidInputBoxes():
			# Make sure control service is ready to receive information
			self.client.wait_for_service()

			# See which gains have changed to send over
			new_gains = self.readInputBoxes()
			for i in range(DOF + 1):
				if GAINS[i] != new_gains[i]:
					self.request.data = "u {} {} {} {}\n".format(
						i, 
						new_gains[i][0], 
						new_gains[i][1], 
						new_gains[i][2])
					self.client.call_async(self.request)
					GAINS[i] = new_gains[i]
					time.sleep(0.3)

			# Save the new gains in a txt file
			self.saveGains(new_gains)
		else:
			QMessageBox.warning(self, "Invalid Entries", "Please make sure each input box has a valid number entered.",
				QMessageBox.Ok, QMessageBox.Ok)

	def closeEvent(self, event):
		""" Tell user the log location """
		QMessageBox.information(self, "Save", "New gains saved in BB's' 'gains' folder.",
			QMessageBox.Ok, QMessageBox.Ok)

def main(args=None):
	rclpy.init(args=args)

	node = Node('control_tuning')

	app = QApplication(sys.argv)
	mainWin = TuningGui()
	mainWin.show()
	sys.exit(app.exec_())

	rclpy.spin(node)
	rclpy.shutdown()