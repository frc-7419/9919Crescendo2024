package frc.robot;



public class JavaDocsExample {
    private double length;
    private double width;

    /**
     * Constructs a new Rectangle with the given length and width.
     * 
     * @param length the length of the rectangle
     * @param width the width of the rectangle
     * @throws IllegalArgumentException if length or width is less than or equal to zero
     */
    public JavaDocsExample(double length, double width) {
        if (length <= 0 || width <= 0) {
            throw new IllegalArgumentException("Length and width must be positive");
        }
        this.length = length;
        this.width = width;
    }

    /**
     * Calculates and returns the area of the rectangle.
     * 
     * @return the area of the rectangle
     */
    public double calculateArea() {
        return length * width;
    }

    /**
     * Calculates and returns the perimeter of the rectangle.
     * 
     * @return the perimeter of the rectangle
     */
    public double calculatePerimeter() {
        return 2 * (length + width);
    }
}