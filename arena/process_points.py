"""Calculate "boundary_lines" of the arena walls."""

from pprint import pprint

ext_pnts_file = "ext_pnts.dat"
int_pnts_file = "int_pnts.dat"

line_list = []

def generate_line_list(pnt_list):
    """Convert a list of points into a list of lines."""
    first_pnt = pnt_list[0]
    line_list = []
    for i in range(len(pnt_list)):
        try:
            line_list.append([pnt_list[i], pnt_list[i+1]])
        except IndexError:
            line_list.append([pnt_list[i], first_pnt])
    return line_list


if __name__ == "__main__":
    """The arena is presumed to be bounded by a fully enclosed exterior
    perimeter wall and a separate loop representing an interior wall.
    Each wall is defined by a series of points (x, y), one at each corner.
    "boundary_lines" go from corner to corner."""

    for file in [ext_pnts_file, int_pnts_file]:
        pnt_list= []

        # Each line of the data file contains x_val, y_val
        with open(file) as f:
            for line in f.readlines():
                x, y = line.split(',')
                pnt_list.append((float(x), float(y)))
            print(f"Processed {len(pnt_list)} points in {file}")

        # Convert pnt_list to line_list and extend
        line_list.extend(generate_line_list(pnt_list))
          

print("Number of lines in line_list: ", len(line_list))
pprint(line_list)
