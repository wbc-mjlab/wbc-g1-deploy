# Gen deploy params (wbc_gen_deploy_params_v2)
#
# Install with:
#   uv run wbc-gen-export --ckpt-path .../model.pt \
#     --output-dir config/policy/gen/<name>/params
#
# Or copy from a training run's params/ after play/export.
#
# Current default (``config.yaml`` ``gen_dir``): ``loco_style`` — ``wbc_loco_style``
# flow_ae with ``cmd_style`` one-hot (stand_walk / crouch / run / sit).
#
# ``model.type`` is read from config.yaml (``flow_ae``, ``cascade_ae``, …).
# Runtime inference is always ONNX (``obs`` → ``reference``); type selects
# metadata / logging only (Euler steps for flow, intent_dim for cascade_ae).
