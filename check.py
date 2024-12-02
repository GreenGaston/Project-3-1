def has_edges(ply_filename):
    """
    Checks if a PLY file contains edges.

    :param ply_filename: The name of the PLY file.
    :return: True if the PLY file contains edges, False otherwise.
    """
    with open(ply_filename, 'r') as ply_file:
        for line in ply_file:
            if line.startswith('element edge'):
                return True
            if line.startswith('end_header'):
                break
    return False

if __name__ == '__main__':
    file1 = "xyzrgb_dragon.ply"
    print(has_edges(file1))  # False