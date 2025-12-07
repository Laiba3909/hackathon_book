# # tests/embeddings_pipeline/integration/test_qdrant_connection.py

# import unittest
# import os
# from unittest.mock import patch
# from src.embeddings_pipeline.qdrant_client import QdrantClientWrapper
# from dotenv import load_dotenv
# load_dotenv()
# import os

# QDRANT_HOST = os.getenv("QDRANT_HOST")
# QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
# class TestQdrantConnection(unittest.TestCase):
#     @patch.dict(os.environ, {"QDRANT_HOST": QDRANT_HOST, "QDRANT_API_KEY": QDRANT_API_KEY})
#     def test_qdrant_client_init_success(self):
#         # This tests that the client can be initialized with environment variables
#         # It does NOT test actual connection to a running Qdrant instance.
#         try:
#             client_wrapper = QdrantClientWrapper()
#             self.assertIsNotNone(client_wrapper.client)
#             # Further checks can be added if QdrantClient has methods to inspect its connection
#         except Exception as e:
#             self.fail(f"QdrantClientWrapper initialization failed: {e}")

#     @patch.dict(os.environ, {"QDRANT_HOST": "", "QDRANT_API_KEY": "test-key"})
#     def test_qdrant_client_init_no_host(self):
#         with self.assertRaisesRegex(ValueError, "QDRANT_HOST and QDRANT_API_KEY must be set"):
#             QdrantClientWrapper()

#     @patch.dict(os.environ, {"QDRANT_HOST": QDRANT_HOST, "QDRANT_API_KEY": QDRANT_API_KEY})
#     def test_qdrant_client_init_no_api_key(self):
#         with self.assertRaisesRegex(ValueError, "QDRANT_HOST and QDRANT_API_KEY must be set"):
#             QdrantClientWrapper()

#     @patch('src.embeddings_pipeline.qdrant_client.QdrantClient')
#     @patch.dict(os.environ, {"QDRANT_HOST": QDRANT_HOST", "QDRANT_API_KEY": QDRANT_API_KEY}
#                              )
#     def test_health_check_success(self, MockQdrantClient):
#         # Mock the QdrantClient's health_check method
#         mock_instance = MockQdrantClient.return_value
#         mock_instance.health_check.return_value = {"status": "green"}

#         client_wrapper = QdrantClientWrapper()
#         result = client_wrapper.health_check()
#         self.assertEqual(result["status"], "ok")
#         self.assertEqual(result["health"], {"status": "green"})
#         mock_instance.health_check.assert_called_once()

#     @patch('src.embeddings_pipeline.qdrant_client.QdrantClient')
#     @patch.dict(os.environ, {"QDRANT_HOST": QDRANT_HOST, "QDRANT_API_KEY": QDRANT_API_KEY})
#     def test_health_check_failure(self, MockQdrantClient):
#         # Mock the QdrantClient's health_check method to raise an exception
#         mock_instance = MockQdrantClient.return_value
#         mock_instance.health_check.side_effect = Exception("Connection error")

#         client_wrapper = QdrantClientWrapper()
#         result = client_wrapper.health_check()
#         self.assertEqual(result["status"], "error")
#         self.assertIn("Connection error", result["message"])
#         mock_instance.health_check.assert_called_once()

# if __name__ == '__main__':
#     unittest.main()


# tests/embeddings_pipeline/integration/test_qdrant_connection.py

import unittest
import os
from unittest.mock import patch
from src.embeddings_pipeline.qdrant_client import QdrantClientWrapper
from dotenv import load_dotenv

# Load .env file
load_dotenv()

# Get environment variables
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

class TestQdrantConnection(unittest.TestCase):
    @patch.dict(os.environ, {"QDRANT_HOST": QDRANT_HOST, "QDRANT_API_KEY": QDRANT_API_KEY})
    def test_qdrant_client_init_success(self):
        try:
            client_wrapper = QdrantClientWrapper()
            self.assertIsNotNone(client_wrapper.client)
        except Exception as e:
            self.fail(f"QdrantClientWrapper initialization failed: {e}")

    @patch.dict(os.environ, {"QDRANT_HOST": "", "QDRANT_API_KEY": "test-key"})
    def test_qdrant_client_init_no_host(self):
        with self.assertRaisesRegex(ValueError, "QDRANT_HOST and QDRANT_API_KEY must be set"):
            QdrantClientWrapper()

    @patch.dict(os.environ, {"QDRANT_HOST": QDRANT_HOST, "QDRANT_API_KEY": ""})
    def test_qdrant_client_init_no_api_key(self):
        with self.assertRaisesRegex(ValueError, "QDRANT_HOST and QDRANT_API_KEY must be set"):
            QdrantClientWrapper()

    @patch('src.embeddings_pipeline.qdrant_client.QdrantClient')
    @patch.dict(os.environ, {"QDRANT_HOST": QDRANT_HOST, "QDRANT_API_KEY": QDRANT_API_KEY})
    def test_health_check_success(self, MockQdrantClient):
        mock_instance = MockQdrantClient.return_value
        mock_instance.health_check.return_value = {"status": "green"}

        client_wrapper = QdrantClientWrapper()
        result = client_wrapper.health_check()
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["health"], {"status": "green"})
        mock_instance.health_check.assert_called_once()

    @patch('src.embeddings_pipeline.qdrant_client.QdrantClient')
    @patch.dict(os.environ, {"QDRANT_HOST": QDRANT_HOST, "QDRANT_API_KEY": QDRANT_API_KEY})
    def test_health_check_failure(self, MockQdrantClient):
        mock_instance = MockQdrantClient.return_value
        mock_instance.health_check.side_effect = Exception("Connection error")

        client_wrapper = QdrantClientWrapper()
        result = client_wrapper.health_check()
        self.assertEqual(result["status"], "error")
        self.assertIn("Connection error", result["message"])
        mock_instance.health_check.assert_called_once()


if __name__ == '__main__':
    unittest.main()
