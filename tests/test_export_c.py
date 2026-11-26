import unittest

import numpy as np
import torch

from scripts.models.export_c import conv_arrays, gru_arrays
from scripts.models.gru_experiment import GRUForecaster
from scripts.models.train_autoencoder import ConvAutoencoder


def conv_forward_like_firmware(a, x):
    """firmware/main/anomaly.cpp's conv forward pass, on the exported arrays."""
    window, features = x.shape
    patch = np.zeros((window, a["conv1_wt"].shape[0]), np.float32)
    for t in range(window):
        for k in range(3):
            if 0 <= t + k - 1 < window:
                patch[t, k * features:(k + 1) * features] = x[t + k - 1]
    h1 = np.maximum(patch @ a["conv1_wt"] + a["conv1_b"], 0)  # [t][c]
    reduced = (window + 1) // 2
    rows = (reduced + 3) // 4 * 4
    q = np.zeros((rows, 3 * 16), np.float32)
    for t in range(reduced):
        for k in range(3):
            if 0 <= 2 * t + k - 1 < window:
                q[t, k * 16:(k + 1) * 16] = h1[2 * t + k - 1]
    h2 = np.maximum(q @ a["conv2_wt"] + a["conv2_b"], 0)[:reduced].flatten()

    def dense(w, b, v):
        padded = np.zeros(w.shape[1], np.float32)
        padded[:len(v)] = v
        return w @ padded + b

    z = dense(a["enc_w"], a["enc_b"], h2)
    hidden = np.maximum(dense(a["dec1_w"], a["dec1_b"], z), 0)
    return dense(a["dec2_w"], a["dec2_b"], hidden)


class ExportCTests(unittest.TestCase):
    def test_conv_layout_matches_torch(self):
        for features in (3, 4):
            torch.manual_seed(features)
            model = ConvAutoencoder(20, features, 32, 8).eval()
            arrays = conv_arrays(model.state_dict(), features, 20)
            for array in arrays.values():
                if array.ndim == 2:
                    self.assertEqual(array.shape[1] % 4, 0)
            x = np.random.default_rng(features).normal(size=(20, features)).astype(np.float32)
            with torch.no_grad():
                expected = model(torch.from_numpy(x.reshape(1, -1)))[0].numpy()
            np.testing.assert_allclose(conv_forward_like_firmware(arrays, x), expected,
                                       atol=1e-5)

    def test_gru_layout_matches_torch(self):
        torch.manual_seed(0)
        model = GRUForecaster(2, 2, 32).eval()
        a = gru_arrays(model.state_dict())
        inputs = np.random.default_rng(0).normal(size=(30, 2)).astype(np.float32)
        with torch.no_grad():
            expected = model(torch.from_numpy(inputs)[None])[0][0].numpy()

        def sigmoid(v):
            return 1 / (1 + np.exp(-v))

        h = np.zeros(32, np.float32)
        outputs = []
        for x in inputs:
            gi = a["w_ih"] @ np.pad(x, (0, 2)) + a["b_ih"]
            gh = a["w_hh"] @ h + a["b_hh"]
            r = sigmoid(gi[:32] + gh[:32])
            z = sigmoid(gi[32:64] + gh[32:64])
            n = np.tanh(gi[64:] + r * gh[64:])
            h = (1 - z) * n + z * h
            outputs.append(a["head_w"] @ h + a["head_b"])
        np.testing.assert_allclose(np.array(outputs), expected, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
