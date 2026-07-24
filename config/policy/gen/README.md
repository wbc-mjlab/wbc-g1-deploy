# Gen deploy params (wbc_gen_deploy_params_v2)
#
# Install with:
#   uv run wbc-gen-export --ckpt-path .../model.pt \
#     --output-dir config/policy/gen/<name>/params
#
# Or copy from a training run's params/ after play/export.
#
# ``model.type`` is read from config.yaml (``flow_ae``, ``cascade_ae``, …).
# Runtime inference is always ONNX (``obs`` → ``reference``); type selects
# metadata / logging only (Euler steps for flow, intent_dim for cascade_ae).
