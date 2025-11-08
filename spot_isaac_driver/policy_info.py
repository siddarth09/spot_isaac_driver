#!/usr/bin/env python3
"""
verify_policy_io.py
--------------------
Utility script to verify TorchScript RL policy input/output structure
for Spot / H1 / any Isaac Lab policy.

Usage:
    python3 verify_policy_io.py --policy policy/spot_policy.pt --obs-dim 48 --act-dim 12
"""

import argparse
import torch
import numpy as np

def main():
    parser = argparse.ArgumentParser(description="Verify TorchScript policy I/O tensors")
    parser.add_argument('--policy', type=str, required=True, help='Path to .pt TorchScript policy')
    parser.add_argument('--obs-dim', type=int, default=48, help='Expected observation vector size')
    parser.add_argument('--act-dim', type=int, default=12, help='Expected action vector size')
    parser.add_argument('--scale', type=float, default=1.0, help='Optional output scaling (e.g., 0.15)')
    args = parser.parse_args()

    print("="*80)
    print(f"🧩 Loading policy: {args.policy}")
    policy = torch.jit.load(args.policy, map_location='cpu')
    policy.eval()
    print("✅ Loaded successfully")

    # Try to infer input signature if scripted
    try:
        print("="*80)
        print("📘 Policy graph structure:\n")
        print(policy.graph)
    except Exception as e:
        print(f"⚠️ Could not print graph: {e}")

    # Create dummy observation tensor
    obs = np.random.uniform(-1, 1, size=(args.obs_dim,)).astype(np.float32)
    obs_t = torch.from_numpy(obs).unsqueeze(0)

    print("="*80)
    print(f"📊 Input tensor shape: {obs_t.shape}")
    print(f"📊 Example observation (first 8): {obs[:8]}")

    # Forward pass
    with torch.no_grad():
        out = policy(obs_t)

    # Tensor info
    print("="*80)
    print("🧮 Raw policy output:")
    print(f"Type: {type(out)}")
    if isinstance(out, torch.Tensor):
        print(f"Shape: {tuple(out.shape)}")
        print(f"Min: {out.min().item():.4f} | Max: {out.max().item():.4f} | Mean: {out.mean().item():.4f}")
        # Squash and scale for visualization
        out_tanh = torch.tanh(out) * args.scale
        print(f"After tanh+scale({args.scale}):")
        print(f"Min: {out_tanh.min().item():.4f} | Max: {out_tanh.max().item():.4f} | Mean: {out_tanh.mean().item():.4f}")
        if out.numel() == args.act_dim:
            print("✅ Output matches expected action dimension.")
        else:
            print(f"❌ Output dim mismatch! Expected {args.act_dim}, got {out.numel()}")
    else:
        print(f"⚠️ Policy output is not a tensor: {out}")

    # Run multiple random passes for stability check
    print("="*80)
    print("🔁 Running 5 random forward passes to check output ranges...")
    mins, maxs = [], []
    for i in range(5):
        obs = np.random.uniform(-1, 1, size=(args.obs_dim,)).astype(np.float32)
        out = policy(torch.from_numpy(obs).unsqueeze(0))
        mins.append(out.min().item())
        maxs.append(out.max().item())
    print(f"Output min range: [{min(mins):.3f}, {max(mins):.3f}]")
    print(f"Output max range: [{min(maxs):.3f}, {max(maxs):.3f}]")

    print("="*80)
    print("✅ Policy verification complete.")
    print("If outputs are extremely large (>10.0) → missing tanh() or scaling inside policy.")
    print("If output dimension mismatch → verify joint order and policy architecture.")
    print("="*80)


if __name__ == "__main__":
    main()
