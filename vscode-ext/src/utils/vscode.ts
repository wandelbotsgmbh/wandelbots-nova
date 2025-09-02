import * as vscode from 'vscode'

/**
 * If list has 0 -> return undefined
 * If list has 1 -> return the single value
 * If list has >1 -> delegate to an async picker
 */
export async function singleOrPick(
  items: string[],
  askToPick: (
    items: string[],
    placeHolder: string,
  ) => Thenable<string | undefined>,
  placeHolder: string,
): Promise<string | undefined> {
  if (items.length === 0) return undefined
  if (items.length === 1) return items[0]
  return askToPick(items, placeHolder)
}

/**
 * Insert text at the current cursor if an editor is active; otherwise just show it
 */
export async function insertOrShow(text: string): Promise<void> {
  const editor = vscode.window.activeTextEditor
  if (editor) {
    await editor.edit((editBuilder) => {
      editBuilder.insert(editor.selection.active, text)
    })
    vscode.window.showInformationMessage(`Robot pose inserted: ${text}`)
  } else {
    vscode.window.showInformationMessage(`Robot pose: ${text}`)
  }
}
