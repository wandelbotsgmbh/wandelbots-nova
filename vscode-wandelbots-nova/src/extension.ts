import * as vscode from 'vscode';

let decorationType: vscode.TextEditorDecorationType | undefined;
let disposables: vscode.Disposable[] = [];

// Simple per-document state for manual toggles:
const manualMarks = new Map<string, Set<number>>(); // key: doc.uri.toString(), value: set of 0-based line numbers

function createDecorationType(context: vscode.ExtensionContext) {
  const showInOverview = vscode.workspace.getConfiguration('lineMarker').get<boolean>('showInOverviewRuler', true);

  const darkIcon = vscode.Uri.joinPath(context.extensionUri, 'media', 'marker-dark.svg');
  const lightIcon = vscode.Uri.joinPath(context.extensionUri, 'media', 'marker-light.svg');

  decorationType?.dispose();
  decorationType = vscode.window.createTextEditorDecorationType({
    gutterIconPath: darkIcon,
    light: { gutterIconPath: lightIcon },
    // Optional, but can help sizing inconsistencies:
    gutterIconSize: 'contain',
    overviewRulerColor: showInOverview ? new vscode.ThemeColor('editorMarkerNavigation.background') : undefined,
    overviewRulerLane: showInOverview ? vscode.OverviewRulerLane.Full : undefined
  });
}

function getRegex(): RegExp | null {
  const enable = vscode.workspace.getConfiguration('lineMarker').get<boolean>('enable', true);
  if (!enable) return null;

  const raw = vscode.workspace.getConfiguration('lineMarker').get<string>('triggerRegex', '');
  if (!raw) return null;

  try {
    return new RegExp(raw, 'i');
  } catch {
    vscode.window.showWarningMessage('[Line Marker] Invalid triggerRegex; ignoring.');
    return null;
  }
}

function collectDecorations(editor: vscode.TextEditor): vscode.DecorationOptions[] {
  const doc = editor.document;
  const opts: vscode.DecorationOptions[] = [];
  const re = getRegex();

  for (let line = 0; line < doc.lineCount; line++) {
    const text = doc.lineAt(line).text;

    // Regex-based markers
    if (re && re.test(text)) {
      opts.push({
        range: new vscode.Range(line, 0, line, 0),
        hoverMessage: new vscode.MarkdownString('**Line Marker** — regex match')
      });
    }

    // Manual markers
    const manual = manualMarks.get(doc.uri.toString());
    if (manual?.has(line)) {
      opts.push({
        range: new vscode.Range(line, 0, line, 0),
        hoverMessage: new vscode.MarkdownString('**Line Marker** — manual')
      });
    }
  }

  return opts;
}

let updateTimer: NodeJS.Timeout | undefined;

function triggerUpdate(editor?: vscode.TextEditor) {
  const debounce = vscode.workspace.getConfiguration('lineMarker').get<number>('debounceMs', 200);
  if (updateTimer) clearTimeout(updateTimer);
  updateTimer = setTimeout(() => {
    const ed = editor ?? vscode.window.activeTextEditor;
    if (!ed || !decorationType) return;
    const decorations = collectDecorations(ed);
    ed.setDecorations(decorationType, decorations);
  }, Math.max(0, debounce));
}

export function activate(context: vscode.ExtensionContext) {
  createDecorationType(context);

  // Toggle a marker on the current line
  context.subscriptions.push(
    vscode.commands.registerCommand('lineMarker.toggleHere', () => {
      const ed = vscode.window.activeTextEditor;
      if (!ed) return;
      const key = ed.document.uri.toString();
      const line = ed.selection.active.line;

      const set = manualMarks.get(key) ?? new Set<number>();
      set.has(line) ? set.delete(line) : set.add(line);
      manualMarks.set(key, set);

      triggerUpdate(ed);
    })
  );

  // Force refresh
  context.subscriptions.push(
    vscode.commands.registerCommand('lineMarker.refresh', () => triggerUpdate())
  );

  // React to editor/document/config changes
  disposables.push(
    vscode.window.onDidChangeActiveTextEditor(() => triggerUpdate()),
    vscode.workspace.onDidChangeTextDocument(e => {
      const ed = vscode.window.activeTextEditor;
      if (ed && e.document.uri.toString() === ed.document.uri.toString()) triggerUpdate(ed);
    }),
    vscode.workspace.onDidChangeConfiguration(e => {
      if (e.affectsConfiguration('lineMarker')) {
        createDecorationType(context);
        triggerUpdate();
      }
    })
  );

  context.subscriptions.push(...disposables);

  // Initial render
  triggerUpdate();
}

export function deactivate() {
  decorationType?.dispose();
  disposables.forEach(d => d.dispose());
}
