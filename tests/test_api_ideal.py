from __future__ import annotations

import unittest

from easytowing.demo_server import _ideal_steering_request_payload


class IdealSteeringApiTests(unittest.TestCase):
    def test_request_accepts_arbitrary_axle_count(self) -> None:
        payload = _ideal_steering_request_payload(
            {
                "id": "five_axle",
                "name": "Five Axle Case",
                "turn_radius_mm": 14000.0,
                "axles": [
                    {"id": f"axle_{index}", "x_mm": index * 1000.0, "y_mm": 0.0, "track_mm": 2500.0}
                    for index in range(5)
                ],
            }
        )

        self.assertEqual(payload["vehicle"]["axle_count"], 5)
        self.assertEqual(len(payload["axles"]), 5)
        self.assertEqual(len(payload["wheel_steering_angles_deg"]), 10)


if __name__ == "__main__":
    unittest.main()
