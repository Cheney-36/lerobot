This tutorial will explain the training script, how to use it, and particularly the use of Hydra to configure everything needed for the training run.

## The training script

LeRobot offers a training script at [`lerobot/scripts/train.py`](../../lerobot/scripts/train.py). At a high level it does the following:

- Loads a Hydra configuration file for the following steps (more on Hydra in a moment).
- Makes a simulation environment.
- Makes a dataset corresponding to that simulation environment.
- Makes a policy.
- Runs a standard training loop with forward pass, backward pass, optimization step, and occasional logging, evaluation (of the policy on the environment), and checkpointing.

## Basics of how we use Hydra

Explaining the ins and outs of [Hydra](https://hydra.cc/docs/intro/) is beyond the scope of this document, but here we'll share the main points you need to know.

First, `lerobot/configs` has a directory structure like this:

```
.
├── default.yaml
├── env
│   ├── aloha.yaml
│   ├── pusht.yaml
│   └── xarm.yaml
└── policy
    ├── act.yaml
    ├── diffusion.yaml
    └── tdmpc.yaml
```

**_For brevity, in the rest of this document we'll drop the leading `lerobot/configs` path. So `default.yaml` really refers to `lerobot/configs/default.yaml`._**

When you run the training script with

```python
python lerobot/scripts/train.py
```

Hydra is set up to read `default.yaml` (via the `@hydra.main` decorator). If you take a look at the `@hydra.main`'s arguments you will see `config_path="../configs", config_name="default"`. At the top of `default.yaml`, is a `defaults` section which looks likes this:

```yaml
defaults:
  - _self_
  - env: pusht
  - policy: diffusion
```

This logic tells Hydra to incorporate configuration parameters from `env/pusht.yaml` and `policy/diffusion.yaml`. _Note: Be aware of the order as any configuration parameters with the same name will be overidden. Thus, `default.yaml` is overridden by `env/pusht.yaml`  which is overidden by `policy/diffusion.yaml`_.

Then, `default.yaml` also contains common configuration parameters such as `device: cuda` or `use_amp: false` (for enabling fp16 training). Some other parameters are set to `???` which indicates that they are expected to be set in additional yaml files. For instance, `training.offline_steps: ???` in `default.yaml` is set to `200000` in `diffusion.yaml`.

Thanks to this `defaults` section in `default.yaml`, if you want to train Diffusion Policy with PushT, you really only need to run:

```bash
python lerobot/scripts/train.py
```

However, you can be more explicit and launch the exact same Diffusion Policy training on PushT with:

```bash
python lerobot/scripts/train.py policy=diffusion env=pusht
```

This way of overriding defaults via the CLI is especially useful when you want to change the policy and/or environment. For instance, you can train ACT on the default Aloha environment with:

```bash
python lerobot/scripts/train.py policy=act env=aloha
```

There are two things to note here:
- Config overrides are passed as `param_name=param_value`.
- Here we have overridden the defaults section. `policy=act` tells Hydra to use `policy/act.yaml`, and `env=aloha` tells Hydra to use `env/aloha.yaml`.

_As an aside: we've set up all of our configurations so that they reproduce state-of-the-art results from papers in the literature._

## Overriding configuration parameters in the CLI

Now let's say that we want to train on a different task in the Aloha environment. If you look in `env/aloha.yaml` you will see something like:

```yaml
# lerobot/configs/env/aloha.yaml
env:
  task: AlohaInsertion-v0
```

And if you look in `policy/act.yaml` you will see something like:

```yaml
# lerobot/configs/policy/act.yaml
dataset_repo_id: lerobot/aloha_sim_insertion_human
```

But our Aloha environment actually supports a cube transfer task as well. To train for this task, you could manually modify the two yaml configuration files respectively.

First, we'd need to switch to using the cube transfer task for the ALOHA environment.

```diff
# lerobot/configs/env/aloha.yaml
env:
-  task: AlohaInsertion-v0
+  task: AlohaTransferCube-v0
```

Then, we'd also need to switch to using the cube transfer dataset.

```diff
# lerobot/configs/policy/act.yaml
-dataset_repo_id: lerobot/aloha_sim_insertion_human
+dataset_repo_id: lerobot/aloha_sim_transfer_cube_human
```

Then, you'd be able to run:

```bash
python lerobot/scripts/train.py policy=act env=aloha
```

and you'd be training and evaluating on the cube transfer task.

An alternative approach to editing the yaml configuration files, would be to override the defaults via the command line:

```bash
python lerobot/scripts/train.py \
    policy=act \
    dataset_repo_id=lerobot/aloha_sim_transfer_cube_human \
    env=aloha \
    env.task=AlohaTransferCube-v0
```

There's something new here. Notice the `.` delimiter used to traverse the configuration hierarchy. _But be aware that the `defaults` section is an exception. As you saw above, we didn't need to write `defaults.policy=act` in the CLI. `policy=act` was enough._

Putting all that knowledge together, here's the command that was used to train https://huggingface.co/lerobot/act_aloha_sim_transfer_cube_human.

```bash
python lerobot/scripts/train.py \
    hydra.run.dir=outputs/train/act_aloha_sim_transfer_cube_human \
    device=cuda
    env=aloha \
    env.task=AlohaTransferCube-v0 \
    dataset_repo_id=lerobot/aloha_sim_transfer_cube_human \
    policy=act \
    training.eval_freq=10000 \
    training.log_freq=250 \
    training.offline_steps=100000 \
    training.save_model=true \
    training.save_freq=25000 \
    eval.n_episodes=50 \
    eval.batch_size=50 \
    wandb.enable=false \
```

There's one new thing here: `hydra.run.dir=outputs/train/act_aloha_sim_transfer_cube_human`, which specifies where to save the training output.

## Using a configuration file not in `lerobot/configs`

Above we discusses the our training script is set up such that Hydra looks for `default.yaml` in `lerobot/configs`. But, if you have a configuration file elsewhere in your filesystem you may use:

```bash
python lerobot/scripts/train.py --config-dir PARENT/PATH --config-name FILE_NAME_WITHOUT_EXTENSION
```

Note: here we use regular syntax for providing CLI arguments to a Python script, not Hydra's `param_name=param_value` syntax.

As a concrete example, this becomes particularly handy when you have a folder with training outputs, and would like to re-run the training. For example, say you previously ran the training script with one of the earlier commands and have `outputs/train/my_experiment/checkpoints/pretrained_model/config.yaml`. This `config.yaml` file will have the full set of configuration parameters within it. To run the training with the same configuration again, do:

```bash
python lerobot/scripts/train.py --config-dir outputs/train/my_experiment/checkpoints/last/pretrained_model --config-name config
```

Note that you may still use the regular syntax for config parameter overrides (eg: by adding `training.offline_steps=200000`).

---

So far we've seen how to train Diffusion Policy for PushT and ACT for ALOHA. Now, what if we want to train ACT for PushT? Well, there are aspects of the ACT configuration that are specific to the ALOHA environments, and these happen to be incompatible with PushT. Therefore, trying to run the following will almost certainly raise an exception of sorts (eg: feature dimension mismatch):

```bash
python lerobot/scripts/train.py policy=act env=pusht dataset_repo_id=lerobot/pusht
```

Please, head on over to our [advanced tutorial on adapting policy configuration to various environments](./advanced/train_act_pusht/train_act_pusht.md) to learn more.

Or in the meantime, happy coding! 🤗
