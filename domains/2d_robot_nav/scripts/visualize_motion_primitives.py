import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_data(file_action_prims_map: str):
    """
    Loads the data from the file
    :param file_action_prims_map: file containing the state discretization vector (x, y, theta) and the discretized and unrounded action prims for each theta value.
    Example:
    '''
    ActionPrimsNumber,12,StateDiscretization,1 1 30
    0,3,5
    1,0,-30
    1,-0,0
    1,-0,30
    0.939693,0.34202,-40
    0.984808,0.173648,-20
    1,-0,0
    0.984808,-0.173648,20
    0.939693,-0.34202,40
    30,3,5
    1,-1,30
    1,-0,-30
    1,-0,0
    0.984808,-0.173648,-40
    0.939693,-0.34202,-20
    0.866025,-0.5,0
    0.766044,-0.642788,20
    0.642788,-0.766044,40
    ...
    '''
    :return: A map from theta values to a list of action primatives and a list containing the state discretization factors (x, y, theta)
    """
    with open(file_action_prims_map, "r") as f:
        header = f.readline().strip().split(",")
        # check if the header is correct
        if header[0] != "ActionPrimsNumber" :
            raise ValueError("Invalid header")
        if header[2] != "StateDiscretization" :
            raise ValueError("Invalid header")
        # get the map index
        num_theta_values = int(header[1])
        state_discretization = header[3].split(" ")
        # get the theta & action prims pairs
        action_prims_map = {}
        # get the paths
        for _ in range(num_theta_values):
            # get the header
            header = f.readline().strip().split(",")
            # get the theta value
            theta = int(header[0])
            # get the number of states
            num_discretized_action_prims = int(header[1])
            num_action_prims = int(header[2])
            # get the action prims
            discretized_action_prims = []
            for _ in range(num_discretized_action_prims):
                # get the action prim
                discretized_action_prim = f.readline().strip().split(",")
                # add the action prim
                discretized_action_prims.append([float(discretized_action_prim[0]), float(discretized_action_prim[1]), float(discretized_action_prim[2])])
            action_prims = []
            for _ in range(num_action_prims):
                # get the action prim
                action_prim = f.readline().strip().split(",")
                # add the action prim
                action_prims.append([float(action_prim[0]), float(action_prim[1]), float(action_prim[2])])
            # add the path
            action_prims_map[theta] = (discretized_action_prims, action_prims)
    # return the data
    return action_prims_map, state_discretization


def visualize(action_prims_map, state_discretization):
    """
    Visualizes each action prim in the map
    :param action_prims_map: A map from theta values to a list of action primitives
    :return:
    """
    # plot the action primitives
    for theta, all_action_prims in action_prims_map.items():
        discretized_action_prims, action_prims = all_action_prims
        # create the figure
        fig = plt.figure()
        plt.title("Action Primitives: State Discretization=" + str(state_discretization) + ", Theta=" + str(theta))
        
        for action_prim in action_prims:
            x = [0, action_prim[0]]
            y = [0, action_prim[1]]
            plt.plot(x, y, color="red", label="Initial Action Prim")

        for discretized_action_prim in discretized_action_prims:
            x = [0, discretized_action_prim[0]]
            y = [0, discretized_action_prim[1]]
            plt.plot(x, y, color="blue", label="Discretized Action Prim")
            
        plt.grid()
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys())

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
    action_prims_map, state_discretization = load_data(args.fileactionprims)
    # visualize the paths
    visualize(action_prims_map, state_discretization)