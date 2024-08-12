

def build_context(model, dataset, input_dict):
    # TODO: implement this context model builder. Currently it is `None`
    # input_dict is already normalized
    context = None
    # if model.context_model is not None:
    #     context = dict()
    #     # (normalized) features of variable environments
    #     if dataset.variable_environment:
    #         env_normalized = input_dict[f'{dataset.field_key_env}_normalized']
    #         context['env'] = env_normalized
    #
    #     # tasks
    #     task_normalized = input_dict[f'{dataset.field_key_task}_normalized']
    #     context['tasks'] = task_normalized
    return context
