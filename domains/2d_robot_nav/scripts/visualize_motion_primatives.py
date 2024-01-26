import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_data(file_action_prims_map: str):
    """
    Loads the data from the file
    :param file_action_prims_map: file containing the log of the action prims map.
    :return: A map from theta values to a list of action primatives
    """
    with open(file_action_prims_map, "r") as f:
        header = f.readline().strip().split(",")
        # check if the header is correct
        if header[0] != "ActionPrimsNumber" :
            raise ValueError("Invalid header")
        # get the map index
        num_theta_values = int(header[3])
        # get the theta & action prims pairs
        action_prims_map = {}
        # get the paths
        for _ in range(num_theta_values):
            # get the header
            header = f.readline().strip().split(",")
            # get the theta value
            theta = int(header[0])
            # get the number of states
            num_action_prims = int(header[1])
            # get the action prims
            action_prims = []
            for _ in range(num_action_prims):
                # get the action prim
                action_prim = f.readline().strip().split(",")
                # add the action prim
                action_prims.append([int(action_prim[0]), int(action_prim[1]), int(action_prim[2])])
            # add the path
            action_prims_map[theta] = action_prims
    # return the data
    return action_prims_map


def visualize(action_prims_map):
    """
    Visualizes each action prim in the map
    :param action_prims_map: A map from theta values to a list of action primatives
    :return:
    """

    # plot the action primatives
    for theta, action_prims in action_prims_map:
        # create the figure
        fig = plt.figure()
        plt.title("Action Primatives when Theta = " + str(theta))
        for action_prim in action_prims:
            x = [0, action_prim[0]]
            y = [0, action_prim[1]]
            plt.plot(x, y)
            
        plt.grid()

        # save the plot to the folder
        if args.folderpath is not None:
            plt.savefig(args.folderpath + "/theta = " + str(theta) )

        # show the plot
        fig.tight_layout()
        plt.show()
        plt.clf()


if __name__ == "__main__":
    # get the path argument
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--fileactionprims", help="path to the file containing the action prims map")
    parser.add_argument("--folderpath", help="path to the folder the generated action prim visuals will be saved to", 
                        default=None)
    args = parser.parse_args()
    # if no argument is provided, raise an error
    if args.fileactionprims is None:
        raise ValueError("No action prims map file provided")

    import os
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))

    # load the data
    action_prims_map = load_data(args.fileactionprims)
    # visualize the paths
    visualize(action_prims_map)