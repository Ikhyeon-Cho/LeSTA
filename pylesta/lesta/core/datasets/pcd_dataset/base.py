import numpy as np
from torch.utils.data import Dataset
import torch


class PCDDatasetBase(Dataset):
    def __init__(self, file_path, cfg):
        """
        Base class for handling PCD data.
        """
        self.cfg = cfg

        # Member variables
        self.points: np.ndarray = None  # shape: (N, num_fields)
        self.point_fields: list = None
        self.label_field: str = None
        self.labels: np.ndarray = None  # [-1, 0, 1], shape: (N,)

        # 1. Read PCD file
        self.points, self.point_fields = self.read_pcd(file_path)

        # 2. Parse labels
        self._parse_labels()

    def _parse_labels(self):
        """Setup label field and indices."""

        self.label_field = self.cfg.get("label_field", self.point_fields[-1])
        if self.label_field not in self.point_fields:
            raise ValueError(
                f"Label field '{self.label_field}' not found in PCD fields")

        label_field_idx = self.point_fields.index(self.label_field)
        self.labels = self.points[:, label_field_idx]

        # Remove label field from point fields
        self.point_fields = [
            f for f in self.point_fields if f != self.label_field]
        point_field_indices = [
            i for i, f in enumerate(self.point_fields) if f != self.label_field]
        self.points = self.points[:, point_field_indices]

    def __len__(self):
        return len(self.points)

    def __getitem__(self, idx):

        point = torch.from_numpy(self.points[idx]).float()
        label = torch.tensor(self.labels[idx], dtype=torch.float32)

        return point, label

        feature_vector = self.feature_vectors[idx]
        label = self.labels[idx]

        # If pseudo labels have been attached, override the original label.
        if self.pseudo_labels is not None:
            pseudo = self.pseudo_labels[idx]
            if pseudo is not None:
                label = pseudo

        return {"feats": feature_vector, "label": label}

    def get_pos_weight(self):
        """
        Get the positive weight for the loss function.
        """
        n_pos = len(self.labels[self.labels == 1])
        n_neg = len(self.labels[self.labels == 0])
        if n_pos == 0:
            return torch.tensor(1.0)
        return torch.tensor(n_neg / n_pos)

    def read_pcd(self, file_path):
        """
        Reads an ASCII .pcd file and extracts the entire data array along with the header fields.
        The header is expected to contain a line like:
            FIELDS x y z elevation step slope roughness curvature variance footprint traversability_label
        Args:
            file_path (str): Path to the .pcd file.
        Returns:
            data (np.ndarray): Array of shape (N, num_fields) with all values.
            fields (list of str): List of field names as given in the header.
        """
        with open(file_path, 'r') as f:
            lines = f.readlines()

        fields = None
        data_start = None
        # Parse the header to extract field names.
        for i, line in enumerate(lines):
            line_stripped = line.strip()
            if line_stripped.upper().startswith("FIELDS"):
                parts = line_stripped.split()
                fields = parts[1:]  # everything after "FIELDS"
            if line_stripped.upper().startswith("DATA"):
                data_start = i + 1
                break

        if data_start is None or fields is None:
            raise ValueError(
                "Missing required header information (FIELDS or DATA) in the PCD file.")

        # Read and convert the data lines.
        data = []
        for line in lines[data_start:]:
            line_stripped = line.strip()
            if not line_stripped:
                continue
            parts = line_stripped.split()
            # Convert each value to float (handling "nan" appropriately)
            values = [float(x) if x.lower() !=
                      "nan" else np.nan for x in parts]
            data.append(values)
        data = np.array(data)

        if data.shape[1] != len(fields):
            raise ValueError(
                f"Mismatch between data columns ({data.shape[1]}) and header fields ({len(fields)})."
            )

        # Remove NaN values
        nan_mask = ~np.isnan(data).any(axis=1)
        data = data[nan_mask]

        return data, fields


if __name__ == "__main__":

    dset = PCDDatasetBase(file_path="/workspace/LeSTA/training_set.pcd",
                          cfg={"feature_fields": ["step", "slope", "roughness", "curvature"],
                               "label_field": "traversability_label"})

    print(dset.feature_fields)
    print(dset.feature_vectors.shape if dset.feature_vectors is not None else None)
    print(dset.labels.shape if dset.labels is not None else None)
    print(len(dset))
    print(dset.points.shape)
    print(dset.point_fields)
    point, label = dset[0]
    print(point)
    print(label)
