from html.parser import HTMLParser

class HTMLStripper(HTMLParser):
    def __init__(self):
        super().__init__()
        self.text_parts = []

    def handle_data(self, d):
        self.text_parts.append(d)

    def get_text(self):
        return ''.join(self.text_parts).strip()

def strip_html(html):
    s = HTMLStripper()
    s.feed(html)
    return s.get_text()