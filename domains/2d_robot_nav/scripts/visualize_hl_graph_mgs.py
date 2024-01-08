import numpy as np
from treelib import Node, Tree


def plot_block_diagram(data: np.array):
    """
    Plots a block diagram of the data

    :param data: a numpy array of shape (n, n) where n is the number of states
    :return:
    """
    tree = Tree()
    open_list = []
    tree.create_node("0", "0")
    open_list.append(0)
    while len(open_list) > 0:
        i = open_list.pop(0)
        for j in range(data.shape[0]):
            if data[i, j] == 1:
                if str(j) not in tree.nodes:
                    tree.create_node(f"{j}", f"{j}", parent=f"{i}")
                    open_list.append(j)
                elif tree.is_ancestor(f"{j}", f"{i}"):
                    continue
                else:
                    # add a duplicate node
                    tree.create_node(f"{j}", f"{j}_{i}", parent=f"{i}")
    print("=========================")
    tree.show()
    print("=========================")


if __name__ == "__main__":
    # load the data
    data = np.loadtxt("../../../cmake-build-debug/hl_graph_mgs.txt", delimiter=",", dtype=int)
    # plot the block diagram
    plot_block_diagram(data)
