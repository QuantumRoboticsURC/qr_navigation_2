import numpy as np

def generate_square_spiral(num_points=100, step_size=1):
    """
    Generate coordinates for a square-like spiral.

    Parameters:
        num_points (int): Number of points in the spiral.
        step_size (float): Distance between turns in the spiral.

    Returns:
        np.ndarray: Array of (x, y) coordinates for the square spiral.
    """
    x, y = 0, 0  # Starting point
    dx, dy = step_size, 0  # Initial direction
    coordinates = [(x, y)]

    steps = 1  # Number of steps before turning
    while len(coordinates) < num_points:
        for _ in range(2):  # Repeat twice to increase steps every full cycle
            for _ in range(steps):
                if len(coordinates) >= num_points:
                    break
                x += dx
                y += dy
                coordinates.append((x, y))
            dx, dy = -dy, dx  # Change direction (90-degree turn)
        steps += 1  # Increase the number of steps after each full cycle

    return np.array(coordinates)

if __name__ == "__main__":
    # Generate the square-like spiral coordinates
    num_points = 20
    step_size = 4
    square_spiral_coordinates = generate_square_spiral(num_points, step_size)

    # Display the result
    print("Square-like spiral coordinates:")
    print(square_spiral_coordinates)
