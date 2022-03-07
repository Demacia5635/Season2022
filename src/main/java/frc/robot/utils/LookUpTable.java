package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class LookUpTable {
    private final List<double[]> table;
    private final int size;

    /**
     * Creates a new look up table with the given size.
     * @param size the amount of interpolated values
     */
    public LookUpTable(int size) {
        table = new ArrayList<double[]>();
        this.size = size + 1;
    }

    public LookUpTable(double[][] table) {
        this.table = new ArrayList<double[]>();
        for (double[] row : table) {
            this.table.add(row);
        }
        size = table[0].length;
    }

    /**
     * adds a new row to the table
     * @param row must be of size specified in constructor
     * @throws IllegalArgumentException if the row is not of the correct size
     */
    public void add(double... row) throws IllegalArgumentException{
        if (row.length != size) {
            throw new IllegalArgumentException("Size of new row (" + row.length + ") does not match row size of: " + size);
        }
        table.add(row);
    }

    /**
     * calculates the interpolated value for the given x value
     * @param value even if not in the table, the value will be interpolated between the closest values
     * @return an array of interpolated values
     */
    public double[] get(double value) {
        double[] ans = new double[size - 1];
        ans[0] = value;

        int length = table.size();
        if (length == 0) {
            return ans;
        }

        if (length == 1) {
            double[] row = table.get(0);
            for(int i = 1; i < size; i++) {
                ans[i - 1] = row[i];
            }
            return ans;
        }

        double[] lower, upper;
        int i;
        for (i = 0; i < length && value > table.get(i)[0]; i++);

        if (i == 0) {
            lower = table.get(0);
            upper = table.get(1);
        } else if (i == length) {
            lower = table.get(length - 2);
            upper = table.get(length - 1);
        } else {
            lower = table.get(i - 1);
            upper = table.get(i);
        }

        for (int j = 1; j < size; j++) {
            ans[j - 1] = lower[j] + (value - lower[0]) * (upper[j] - lower[j]) / (upper[0] - lower[0]);
        }
        return ans;
    }
}
