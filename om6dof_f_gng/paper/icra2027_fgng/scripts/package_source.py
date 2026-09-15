#!/usr/bin/env python3
"""Package only reviewed manuscript/figure files, excluding local experiment data."""
from pathlib import Path
import zipfile,hashlib,json
p=Path(__file__).resolve().parents[1]
files=[p/x for x in ['main.tex','main.pdf','references.bib','main.bbl','IEEEtran.bst','numbers.tex','tables.tex','protocol_details.tex','results.tex','additional_results.tex','rotation_table.tex','compile_tex.sh','README_SOURCE.md','verification.json']]
files+=sorted((p/'figures').glob('*.pdf'))+sorted((p/'figures').glob('*.svg'))+sorted((p/'figures').glob('*.png'))
files += [p/'template/ieeeconf.cls',p/'template/IEEEtran.bst',p/'template/provenance.json']
with zipfile.ZipFile(p/'paper_source.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for f in files:z.write(f,'wa_fgng_icra2027/'+f.relative_to(p).as_posix())
manifest={'archive':'paper_source.zip','sha256':hashlib.sha256(p.joinpath('paper_source.zip').read_bytes()).hexdigest(),'files':{f.relative_to(p).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for f in files},'scope':'LaTeX, completed numerical tables, and rendered figures; compile without dataset'}
p.joinpath('source_archive_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Source archive:',p.joinpath('paper_source.zip').stat().st_size,'bytes')
