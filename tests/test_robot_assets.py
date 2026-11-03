"""Exercise the existing HTTP handler without a socket, hardware or live service."""
import io
import unittest
from types import SimpleNamespace

from ur_pick_place.vision.teach_ui import Handler


class RobotAssets(unittest.TestCase):
    def request(self, path, host='127.0.0.1:8766'):
        handler = Handler.__new__(Handler)
        handler.server = SimpleNamespace(service=object(), token='offline-test', allowed_hosts={'127.0.0.1:8766'})
        handler.path, handler.headers, handler.wfile = path, {'Host': host}, io.BytesIO()
        result = {'headers': {}}
        handler.send_response = lambda status: result.update(status=status)
        handler.send_header = lambda key, value: result['headers'].update({key: value})
        handler.end_headers = lambda: None
        handler.do_GET()
        return result, handler.wfile.getvalue()

    def test_assets_use_existing_handler_and_security_policy(self):
        for path in ('/tool-envelope.html', '/robot-preview.js', '/ur10e-model.js', '/UR-MODEL-NOTICE.txt'):
            result, body = self.request(path)
            self.assertEqual(result['status'], 200)
            self.assertGreater(len(body), 50)
            self.assertIn("script-src 'self'", result['headers']['Content-Security-Policy'])
            self.assertNotIn('unsafe-inline', result['headers']['Content-Security-Policy'])
        self.assertIn(b'robot-preview.js', self.request('/tool-envelope.html')[1])

    def test_invalid_host_stays_blocked(self):
        self.assertEqual(self.request('/ur10e-model.js', 'untrusted.example')[0]['status'], 403)


if __name__ == '__main__':
    unittest.main()
