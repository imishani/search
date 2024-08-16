

class FittedQNetLoss:

    def __init__(self):
        pass

    @staticmethod
    def loss_fn(model, input_dict, dataset, step=None):
        """
        Loss function for training fitted q network
        """
        data_normalized = input_dict[f'{dataset.field_key_data}_normalized']
        # obsercation is the state and the goal
        ob = data_normalized[:, :4]
        data_denormalized = dataset.denormalize(data_normalized, key=dataset.field_key_data)
        ac = data_denormalized[:, 4]
        # convert the torch objects to ints
        ac = ac.long()
        nex_ob = data_normalized[:, 5:7]
        rew = data_normalized[:, 7]
        q_next = data_normalized[:, 8]
        loss, info = model.update(ob, ac, nex_ob, rew, q_next)

        loss_dict = {'fqi_loss': loss}

        return loss_dict, info
