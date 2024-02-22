import csv
import numpy

class csvToMatrix:
    def __init__(self, mapRoute):
        # Initialize the csvToMatrix object with the mapRoute parameter
        self.mapRoute = mapRoute

    def transform(self):
        # Open the CSV file using the mapRoute parameter and read it as text
        reader = csv.reader(open(self.mapRoute, "rt"), delimiter=",")

        # Convert the CSV data into a list of lists
        x = list(reader)

        # Convert the list of lists into a numpy array of integers
        result = numpy.array(x).astype("int")

        # Return the resulting numpy array
        return result