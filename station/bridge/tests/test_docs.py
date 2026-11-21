"""Docs must match the headers: link protocol version and the param table."""
import os
import re
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def _read(*parts):
    with open(os.path.join(ROOT, *parts), encoding="utf-8") as f:
        return f.read()


def _param_names():
    return re.findall(r'^\s*X\(\w+,\s*"([^"]+)"', _read("src", "params.h"), re.M)


def _doc_names():
    """Names in docs/params.md's Params table; 'a.x / y' also names the param a.*y that ends with it."""
    txt = _read("docs", "params.md")
    sec = txt.split("## Params", 1)[1].split("\n## ", 1)[0]
    names = set()
    for line in sec.splitlines():
        m = re.match(r"\|\s*([a-z]+\.[^|]+?)\s*\|", line)
        if not m:
            continue
        prefix = ""
        for part in (p.strip() for p in m.group(1).split("/")):
            if "." in part:
                prefix = part.split(".")[0] + "."
                names.add(part)
            else:
                hit = [n for n in _param_names() if n.startswith(prefix) and n.endswith(("_" + part, "." + part))]
                names.update(hit or [prefix + part])
    return names


class DocsMatchHeaders(unittest.TestCase):
    def test_link_version_in_docs(self):
        ver = re.search(r"#define\s+LINK_PROTOCOL_VERSION\s+(\d+)",
                        _read("include", "protocol", "link_protocol.h")).group(1)
        for doc in (("README.md",), ("station", "README.md")):
            txt = _read(*doc)
            found = re.findall(r"link(?: protocol)? v(\d+)", txt)
            self.assertTrue(found, "/".join(doc) + " does not mention the link protocol version")
            self.assertEqual(set(found), {ver}, "/".join(doc) + " has a stale link version")

    def test_param_count_in_readme(self):
        m = re.search(r"params (\d+)", _read("README.md"))
        self.assertIsNotNone(m)
        self.assertEqual(int(m.group(1)), len(_param_names()))

    def test_param_table(self):
        src, doc = set(_param_names()), _doc_names()
        self.assertEqual(sorted(src - doc), [], "params missing from docs/params.md")
        self.assertEqual(sorted(doc - src), [], "removed/renamed params still in docs/params.md")


if __name__ == "__main__":
    unittest.main()