#!/usr/bin/env python3
"""Mechanical checks; this is not a scientific acceptance or novelty assessment."""
from pathlib import Path
import json,re,subprocess,hashlib
p=Path(__file__).resolve().parents[1]
log=(p/'main.log').read_text(errors='replace')
text=subprocess.check_output(['pdftotext',str(p/'main.pdf'),'-'],text=True)
info=subprocess.check_output(['pdfinfo',str(p/'main.pdf')],text=True)
pages=int(re.search(r'^Pages:\s+(\d+)',info,re.M).group(1))
checks={'page_limit_8':pages<=8,'pages':pages,
 'no_undefined_citations':'undefined' not in log.lower(),
 'no_overfull_boxes':'Overfull' not in log,
 'no_text_placeholders':not any(x in text for x in ['will be populated','will be generated','PLACEHOLDER','TODO']),
 'anonymous_author_line':'Anonymous authors' in text,
 'ai_disclosure':'AIDISCLOSURE' in re.sub(r'\s+', '', text).upper() and 'OpenAI Codex' in text,
 'no_local_identity_in_pdf':not any(x in text for x in ['/home/','kublab@']),
 'pdf_sha256':hashlib.sha256((p/'main.pdf').read_bytes()).hexdigest(),
 'note':'Checks verify format and internal artifacts only. They do not establish scientific novelty, author approval, or conference acceptance.'}
(p/'verification.json').write_text(json.dumps(checks,indent=2)+'\n')
print(json.dumps(checks,indent=2))
assert all(v for k,v in checks.items() if isinstance(v,bool)),checks
