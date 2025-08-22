import path from 'path'
import { fileURLToPath } from 'url'

const __filename = fileURLToPath(import.meta.url)
const __dirname = path.dirname(__filename)

/** @type {import('webpack').Configuration} */
export default {
  target: 'node',
  mode: 'production',
  entry: './src/extension.ts',
  output: {
    path: path.resolve(__dirname, 'dist'),
    filename: 'extension.js',
    libraryTarget: 'module',
    module: true,
  },
  experiments: { outputModule: true },
  externals: {
    vscode: 'commonjs vscode',
    '@wandelbots/nova-js': 'commonjs @wandelbots/nova-js',
    ws: 'module ws',
  },
  resolve: { extensions: ['.ts', '.js'] },
  module: {
    rules: [{ test: /\.ts$/, use: 'ts-loader', exclude: /node_modules/ }],
  },
  devtool: 'source-map',
}
