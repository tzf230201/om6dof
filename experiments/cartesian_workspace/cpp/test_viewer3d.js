#!/usr/bin/env node
'use strict';

// Execute the generated viewer, including its actual application JavaScript,
// without ROS, a network connection, third-party packages, or a GUI session.
// A real browser is still needed to validate canvas rendering visually.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const {test} = require('node:test');

const filename = process.argv[2];
if (!filename) {
  process.stderr.write('Usage: node test_viewer3d.js /path/to/viewer3d.html\n');
  process.exit(2);
}
const html = fs.readFileSync(filename, 'utf8');
const blocks = Array.from(html.matchAll(/<script\b([^>]*)>([\s\S]*?)<\/script>/gi));
const dataBlock = blocks.find(match => /id=["']workspace-data["']/.test(match[1]));
assert.ok(dataBlock, 'viewer must embed its data to work offline');
const sourceData = JSON.parse(dataBlock[2]);
const application = blocks.filter(match => !/type=["']application\/json["']/.test(match[1]))
  .map(match => match[2]).join('\n');
assert.ok(application.trim(), 'viewer must contain executable application code');
new vm.Script(application, {filename});

function mockDocument(markup, payload) {
  const elements = [];
  const handlers = new Map();
  const drawing = [];
  const context = new Proxy({
    measureText: text => ({width: String(text).length * 7}),
  }, {
    get(target, name) {
      if (name in target) return target[name];
      return (...args) => drawing.push({name, args});
    },
  });
  class Element {
    constructor(tag, attributes = {}) {
      this.tagName = tag.toUpperCase();
      this.attributes = attributes;
      this.id = attributes.id || '';
      this.className = attributes.class || '';
      this.value = attributes.value || '';
      this.checked = Object.hasOwn(attributes, 'checked');
      this.disabled = Object.hasOwn(attributes, 'disabled');
      this.hidden = Object.hasOwn(attributes, 'hidden');
      this.dataset = Object.fromEntries(Object.entries(attributes)
        .filter(([key]) => key.startsWith('data-'))
        .map(([key, value]) => [key.slice(5).replace(/-([a-z])/g, (_, c) => c.toUpperCase()), value]));
      this.style = {};
      this.children = [];
      this.events = new Map();
      this.textContent = '';
      this._innerHTML = '';
      this.clientWidth = this.width = 1000;
      this.clientHeight = this.height = 650;
      const classes = new Set(this.className.split(/\s+/).filter(Boolean));
      this.classList = {
        add: (...values) => values.forEach(value => classes.add(value)),
        remove: (...values) => values.forEach(value => classes.delete(value)),
        contains: value => classes.has(value),
        toggle: (value, force) => {
          const enabled = force === undefined ? !classes.has(value) : force;
          if (enabled) classes.add(value); else classes.delete(value);
          return enabled;
        },
      };
      elements.push(this);
    }
    get innerHTML() { return this._innerHTML; }
    set innerHTML(value) {
      this._innerHTML = String(value);
      this.children = parse(String(value));
    }
    get options() { return this.children.filter(child => child.tagName === 'OPTION'); }
    appendChild(child) {
      this.children.push(child);
      child.parentElement = this;
      if (this.tagName === 'SELECT' && this.children.length === 1) this.value = child.value;
      return child;
    }
    append(...children) { children.forEach(child => this.appendChild(child)); }
    replaceChildren(...children) { this.children = []; this.append(...children); }
    addEventListener(name, callback) {
      if (!this.events.has(name)) this.events.set(name, []);
      this.events.get(name).push(callback);
    }
    dispatchEvent(event) {
      event.target ||= this;
      for (const callback of this.events.get(event.type) || []) callback(event);
      return true;
    }
    getAttribute(name) { return this.attributes[name] ?? null; }
    setAttribute(name, value) { this.attributes[name] = String(value); }
    removeAttribute(name) { delete this.attributes[name]; }
    getBoundingClientRect() { return {left: 0, top: 0, width: 1000, height: 650, right: 1000, bottom: 650}; }
    getContext(kind) { assert.equal(kind, '2d'); return context; }
    querySelectorAll(selector) { return this.children.filter(element => matches(element, selector)); }
    querySelector(selector) { return this.querySelectorAll(selector)[0] || null; }
    setPointerCapture() {}
    releasePointerCapture() {}
    focus() {}
  }
  function parse(value) {
    return Array.from(value.matchAll(/<([a-z][a-z0-9-]*)\b([^>]*)>/gi), match => {
      const attributes = {};
      for (const attr of match[2].matchAll(/([a-zA-Z_:][\w:.-]*)(?:\s*=\s*(?:"([^"]*)"|'([^']*)'|([^\s>]+)))?/g)) {
        attributes[attr[1]] = attr[2] ?? attr[3] ?? attr[4] ?? '';
      }
      return new Element(match[1], attributes);
    });
  }
  function matches(element, selector) {
    if (selector.includes(',')) return selector.split(',').some(part => matches(element, part.trim()));
    const id = selector.match(/#([\w-]+)/);
    if (id && element.id !== id[1]) return false;
    const tag = selector.match(/^[a-z][\w-]*/i);
    if (tag && element.tagName !== tag[0].toUpperCase()) return false;
    for (const className of selector.matchAll(/\.([\w-]+)/g)) {
      if (!element.classList.contains(className[1])) return false;
    }
    for (const attr of selector.matchAll(/\[([\w-]+)(?:=["']?([^\]"']+)["']?)?\]/g)) {
      if (!Object.hasOwn(element.attributes, attr[1])) return false;
      if (attr[2] !== undefined && element.attributes[attr[1]] !== attr[2]) return false;
    }
    if (selector.includes(':checked') && !element.checked) return false;
    return true;
  }
  parse(markup);
  const document = {
    elements,
    drawing,
    body: elements.find(element => element.tagName === 'BODY'),
    documentElement: elements.find(element => element.tagName === 'HTML'),
    getElementById: id => elements.find(element => element.id === id) || null,
    querySelector: selector => elements.find(element => matches(element, selector)) || null,
    querySelectorAll: selector => elements.filter(element => matches(element, selector)),
    createElement: tag => new Element(tag),
    createTextNode: text => ({textContent: String(text)}),
    addEventListener: (name, callback) => handlers.set(name, callback),
  };
  document.getElementById('workspace-data').textContent = JSON.stringify(payload);
  return document;
}

function startViewer(payload = sourceData) {
  const document = mockDocument(html, payload);
  const frames = [];
  const sandbox = {
    document, console, devicePixelRatio: 1, innerWidth: 1440, innerHeight: 1000,
    performance: {now: () => 0},
    requestAnimationFrame: callback => { frames.push(callback); return frames.length; },
    cancelAnimationFrame: () => {},
    addEventListener: () => {},
    matchMedia: () => ({matches: false, addEventListener: () => {}}),
    ResizeObserver: class {observe() {} disconnect() {}},
    fetch: () => { throw new Error('Viewer attempted a network request'); },
    XMLHttpRequest: class {constructor() { throw new Error('Viewer attempted XMLHttpRequest'); }},
    setTimeout: callback => { frames.push(callback); return frames.length; },
    clearTimeout: () => {},
  };
  sandbox.window = sandbox;
  sandbox.self = sandbox;
  vm.runInNewContext(application, sandbox, {filename, timeout: 3000});
  // A queued redraw should settle; this viewer does not need an animation loop.
  for (let i = 0; frames.length && i < 20; ++i) frames.shift()(0);
  const viewer = sandbox.__workspaceViewer;
  assert.ok(viewer, 'diagnostic API must exist');
  return {viewer, document};
}

const fixturePoints = [
  [-50, 0, 50, 0, .123, 12.5, .2, .3, 1, 2, 3, 4, 5, 6, 10, 20, 30],
  [0, 0, 50, 1, .001, .3, .4, .5, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [50, 50, 0, 2, .05, 3, .8, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  [0, -50, 0, 3, null, null, null, null, null, null, null, null, null, null, 0, 0, 0],
  [50, 0, -50, 4, null, null, null, null, null, null, null, null, null, null, 0, 0, 0],
];
const fixture = {...sourceData, points: fixturePoints};
const visible = viewer => Array.from(viewer.getState().visibleIndices).sort((a, b) => a - b);
const roughlyEqual = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-8,
  `expected ${actual} to equal ${expected}`);
function dispatch(element, type, properties = {}) {
  const event = {type, prevented: false, preventDefault() { this.prevented = true; }, ...properties};
  element.dispatchEvent(event);
  return event;
}

test('standalone HTML has valid JavaScript and no remote scripts, imports or fetch calls', () => {
  assert.doesNotMatch(html, /<script\b[^>]*\bsrc\s*=/i);
  assert.doesNotMatch(application, /\b(fetch|XMLHttpRequest|WebSocket|importScripts)\s*\(/);
  assert.doesNotMatch(application, /\bimport\s*(?:\(|[\w{*])/);
  assert.ok(sourceData.points.length > 0);
});

test('generated dataset initializes and draws all samples without a network connection', () => {
  const {viewer, document} = startViewer();
  viewer.reset();
  viewer.render();
  assert.equal(visible(viewer).length, sourceData.points.length);
  assert.ok(document.drawing.length > 0, 'canvas must receive drawing commands');
});

test('canvas markers and legend use the requested color/shape mapping', () => {
  const shapes = ['circle', 'triangle', 'square', 'diamond', 'cross'];
  const colors = ['#15976b', '#397ac6', '#e5a21a', '#9658ad', '#d15b59'];
  const paths = [
    ['beginPath', 'arc', 'fill'],
    ['beginPath', 'moveTo', 'lineTo', 'lineTo', 'closePath', 'fill'],
    ['beginPath', 'rect', 'fill'],
    ['beginPath', 'moveTo', 'lineTo', 'lineTo', 'lineTo', 'closePath', 'fill'],
    ['beginPath', 'moveTo', 'lineTo', 'moveTo', 'lineTo', 'stroke'],
  ];
  assert.deepEqual(Array.from(html.matchAll(/<svg class="marker"[^>]*data-shape="([^"]+)"/g), m => m[1]), shapes);
  for (let status = 0; status < 5; status++) {
    const {viewer, document} = startViewer({...fixture, points: [fixturePoints[status]]});
    document.drawing.length = 0;
    viewer.render();
    const commands = document.drawing.map(command => command.name);
    assert.deepEqual(commands.slice(commands.lastIndexOf('beginPath')), paths[status]);
    const ctx = document.getElementById('workspace-canvas').getContext('2d');
    assert.equal(ctx.fillStyle, colors[status]);
    assert.equal(ctx.strokeStyle, colors[status]);
  }
});

test('generated data exposes intermediate sampled planes without thinning points', () => {
  const {viewer, document} = startViewer();
  for (const [axisIndex, axis] of ['x', 'y', 'z'].entries()) {
    const levels = [...new Set(sourceData.points.map(point => point[axisIndex]))].sort((a,b) => a-b);
    const value = levels.find(level => level > 0) ?? levels[0];
    assert.equal(Number(document.getElementById(`slice-value-${axis}`).max), levels.length - 1);
    viewer.setSlice(axis, value);
    const expected = sourceData.points.flatMap((point, index) => Math.abs(point[axisIndex] - value) < 1e-6 ? [index] : []);
    assert.deepEqual(visible(viewer), expected);
    viewer.setSlice(axis, null);
  }
  assert.equal(visible(viewer).length, sourceData.points.length);
});

test('large triangle tips remain selectable at the largest point-size setting', () => {
  const {viewer, document} = startViewer({...fixture, points: [fixturePoints[1]]});
  document.getElementById('point-size').value = '8';
  viewer.render();
  const at = viewer.projectPoint(fixturePoints[1]);
  const canvas = document.getElementById('workspace-canvas');
  const pointer = {pointerId: 1, clientX: at.x, clientY: at.y - 10, button: 0};
  dispatch(canvas, 'pointerdown', pointer);
  dispatch(canvas, 'pointerup', pointer);
  assert.equal(viewer.getState().selectedIndex, 0);
});

test('each status category independently filters the correct points', () => {
  const {viewer} = startViewer(fixture);
  viewer.reset();
  for (let category = 0; category < 5; ++category) {
    viewer.setCategory(category, false);
    assert.deepEqual(visible(viewer), fixturePoints.map((_, i) => i).filter(i => i !== category));
    viewer.setCategory(category, true);
    assert.deepEqual(visible(viewer), [0, 1, 2, 3, 4]);
  }
});

test('X, Y and Z constant slices intersect correctly with category filters', () => {
  const {viewer} = startViewer(fixture);
  viewer.reset();
  viewer.setSlice('x', 0);
  assert.deepEqual(visible(viewer), [1, 3]);
  viewer.setSlice('x', null);
  viewer.setSlice('y', 0);
  assert.deepEqual(visible(viewer), [0, 1, 4]);
  viewer.setSlice('z', 50);
  assert.deepEqual(visible(viewer), [0, 1]);
  viewer.setCategory(1, false);
  assert.deepEqual(visible(viewer), [0]);
  viewer.setSlice('x', 50);
  assert.deepEqual(visible(viewer), []);
  viewer.reset();
  assert.deepEqual(visible(viewer), [0, 1, 2, 3, 4]);
});

test('isometric projection keeps the three positive axes distinct', () => {
  const {viewer} = startViewer(fixture);
  viewer.setView('iso');
  const origin = viewer.projectPoint([0, 0, 0]);
  const axes = [[50, 0, 0], [0, 50, 0], [0, 0, 50]].map(point => viewer.projectPoint(point));
  axes.forEach(axis => {
    assert.ok(Number.isFinite(axis.x) && Number.isFinite(axis.y) && Number.isFinite(axis.depth));
    assert.ok(Math.hypot(axis.x - origin.x, axis.y - origin.y) > 1);
  });
  for (let i = 0; i < 3; ++i) {
    for (let j = i + 1; j < 3; ++j) {
      assert.ok(Math.hypot(axes[i].x - axes[j].x, axes[i].y - axes[j].y) > 1);
    }
  }
});

test('orthogonal XY, XZ and YZ view presets suppress only the normal coordinate', () => {
  const {viewer} = startViewer(fixture);
  for (const [name, normal] of [['xy', [0, 0, 50]], ['xz', [0, 50, 0]], ['yz', [50, 0, 0]]]) {
    viewer.setView(name);
    const origin = viewer.projectPoint([0, 0, 0]);
    const projected = viewer.projectPoint(normal);
    roughlyEqual(projected.x, origin.x);
    roughlyEqual(projected.y, origin.y);
    assert.ok(Math.abs(projected.depth - origin.depth) > 1);
    assert.equal(viewer.getState().preset, name);
  }
});

test('selection keeps the original point index and exact recorded values', () => {
  const {viewer, document} = startViewer(fixture);
  viewer.reset();
  viewer.selectPoint(0);
  assert.equal(viewer.getState().selectedIndex, 0);
  const displayed = document.elements.map(element => element.textContent + element.innerHTML).join('\n');
  assert.match(displayed, /-50/);
  assert.match(displayed, /12\.5/);
  const details = document.getElementById('selection-details').innerHTML;
  assert.match(details, /-50\.0 \/ 0\.0 \/ 50\.0/);
  assert.match(details, /0\.123000/);
  assert.match(details, /10\.00 \/ 20\.00 \/ 30\.00/);
  assert.match(details, /57\.30, 114\.59, 171\.89, 229\.18, 286\.48, 343\.77/);
  // Selecting a filtered point must not silently select a different CSV record.
  viewer.setCategory(0, false);
  assert.ok(viewer.getState().selectedIndex === null || viewer.getState().selectedIndex === 0);
});

test('reset restores filters, selection and isometric camera deterministically', () => {
  const {viewer} = startViewer(fixture);
  viewer.reset();
  const initial = viewer.getState();
  viewer.setView('xy');
  viewer.setSlice('z', 50);
  viewer.setCategory(1, false);
  viewer.selectPoint(0);
  viewer.reset();
  const after = viewer.getState();
  assert.deepEqual(visible(viewer), [0, 1, 2, 3, 4]);
  assert.equal(after.selectedIndex, initial.selectedIndex);
  for (const key of ['azimuth', 'elevation', 'zoom', 'preset']) assert.equal(after[key], initial[key]);
});

test('real control events update categories, sampled slice coordinates and camera buttons', () => {
  const {viewer, document} = startViewer(fixture);
  const element = id => document.getElementById(id);
  dispatch(element('show-poses'), 'click');
  assert.deepEqual(visible(viewer), [0, 1]);
  dispatch(element('show-all'), 'click');
  element('status-1').checked = false;
  dispatch(element('status-1'), 'change');
  assert.deepEqual(visible(viewer), [0, 2, 3, 4]);
  element('slice-value-z').value = '2'; // Sorted sampled levels: -50, 0, 50.
  element('slice-enable-z').checked = true;
  dispatch(element('slice-enable-z'), 'change');
  assert.deepEqual(visible(viewer), [0]);
  assert.equal(element('slice-label-z').textContent, '50 mm');
  element('slice-value-z').value = '0';
  dispatch(element('slice-value-z'), 'input');
  assert.deepEqual(visible(viewer), [4]);
  assert.equal(element('slice-label-z').textContent, '-50 mm');
  dispatch(element('clear-slices'), 'click');
  assert.deepEqual(visible(viewer), [0, 2, 3, 4]);
  dispatch(element('view-yz'), 'click');
  assert.equal(viewer.getState().preset, 'yz');
  assert.equal(element('view-yz').getAttribute('aria-pressed'), 'true');
  assert.equal(element('view-iso').getAttribute('aria-pressed'), 'false');
  dispatch(element('reset'), 'click');
  assert.deepEqual(visible(viewer), [0, 1, 2, 3, 4]);
  assert.equal(viewer.getState().preset, 'iso');
});

test('canvas clicks inspect the clicked point and Escape or clear-selection removes it', () => {
  const {viewer, document} = startViewer(fixture);
  const canvas = document.getElementById('workspace-canvas');
  const at = viewer.projectPoint(fixturePoints[0]);
  const pointer = {pointerId: 1, clientX: at.x, clientY: at.y, button: 0};
  dispatch(canvas, 'pointerdown', pointer);
  dispatch(canvas, 'pointerup', pointer);
  assert.equal(viewer.getState().selectedIndex, 0);
  assert.equal(document.getElementById('selection-details').hidden, false);
  assert.equal(dispatch(canvas, 'keydown', {key: 'Escape'}).prevented, true);
  assert.equal(viewer.getState().selectedIndex, null);
  viewer.selectPoint(4);
  const details = document.getElementById('selection-details').innerHTML;
  assert.match(details, /Position unresolved/);
  assert.match(details, /n\/a, n\/a, n\/a, n\/a, n\/a, n\/a/);
  dispatch(document.getElementById('clear-selection'), 'click');
  assert.equal(viewer.getState().selectedIndex, null);
  assert.equal(document.getElementById('selection-details').hidden, true);
});

test('pointer orbit, right-button pan and pointer cancellation have separate effects', () => {
  const {viewer, document} = startViewer(fixture);
  const canvas = document.getElementById('workspace-canvas');
  const initial = viewer.getState();
  dispatch(canvas, 'pointerdown', {pointerId: 1, clientX: 100, clientY: 100, button: 0});
  dispatch(canvas, 'pointermove', {pointerId: 1, clientX: 130, clientY: 120});
  dispatch(canvas, 'pointerup', {pointerId: 1, clientX: 130, clientY: 120});
  roughlyEqual(viewer.getState().azimuth, initial.azimuth - .24);
  roughlyEqual(viewer.getState().elevation, initial.elevation + .16);
  assert.equal(viewer.getState().selectedIndex, null);
  const beforePan = viewer.projectPoint([0, 0, 0]);
  const angle = viewer.getState().azimuth;
  dispatch(canvas, 'pointerdown', {pointerId: 2, clientX: 100, clientY: 100, button: 2});
  dispatch(canvas, 'pointermove', {pointerId: 2, clientX: 120, clientY: 90});
  dispatch(canvas, 'pointercancel');
  const afterPan = viewer.projectPoint([0, 0, 0]);
  roughlyEqual(afterPan.x, beforePan.x + 20);
  roughlyEqual(afterPan.y, beforePan.y - 10);
  assert.equal(viewer.getState().azimuth, angle);
  dispatch(canvas, 'pointermove', {pointerId: 2, clientX: 400, clientY: 400});
  roughlyEqual(viewer.projectPoint([0, 0, 0]).x, afterPan.x);
  roughlyEqual(viewer.projectPoint([0, 0, 0]).y, afterPan.y);
});

test('wheel and keyboard zoom are bounded; arrow keys rotate without selecting a sample', () => {
  const {viewer, document} = startViewer(fixture);
  const canvas = document.getElementById('workspace-canvas');
  assert.equal(dispatch(canvas, 'wheel', {deltaY: -100}).prevented, true);
  assert.ok(viewer.getState().zoom > 1);
  dispatch(canvas, 'wheel', {deltaY: 1e9});
  assert.equal(viewer.getState().zoom, .25);
  dispatch(canvas, 'wheel', {deltaY: -1e9});
  assert.equal(viewer.getState().zoom, 8);
  dispatch(canvas, 'keydown', {key: '+'});
  assert.equal(viewer.getState().zoom, 8);
  dispatch(canvas, 'keydown', {key: '-'});
  assert.ok(viewer.getState().zoom < 8);
  const angle = viewer.getState().azimuth;
  assert.equal(dispatch(canvas, 'keydown', {key: 'ArrowRight'}).prevented, true);
  assert.notEqual(viewer.getState().azimuth, angle);
  assert.equal(viewer.getState().selectedIndex, null);
  assert.equal(dispatch(canvas, 'keydown', {key: 'a'}).prevented, false);
});
