#!/usr/bin/env python3

'''


'''
import sys
import h5py

def print_h5_tree(group, indent=""):
    """
    Recursively print the contents of an HDF5 group/dataset
    using an ASCII-tree style with '|' and '-'.
    """
    for name, item in group.items():
        if isinstance(item, h5py.Group):
            # print group name with trailing '/'
            print(f"{indent}|-- {name}/")
            # increase indent for children
            print_h5_tree(item, indent + "|   ")
        elif isinstance(item, h5py.Dataset):
            # print dataset name, shape, and dtype
            print(f"{indent}|-- {name}  (shape={item.shape}, dtype={item.dtype})")
        else:
            # other HDF5 object types
            print(f"{indent}|-- {name}  ({type(item)})")

def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <file.h5>")
        sys.exit(1)

    filename = sys.argv[1]
    try:
        with h5py.File(filename, 'r') as f:
            print(f"Inspecting HDF5 file: {filename}\n/")
            print_h5_tree(f)
    except (OSError, IOError) as e:
        print(f"Error opening '{filename}': {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
