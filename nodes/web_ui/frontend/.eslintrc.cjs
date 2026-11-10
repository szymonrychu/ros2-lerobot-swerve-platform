module.exports = {
  root: true,
  env: { browser: true, es2022: true },
  parser: '@typescript-eslint/parser',
  parserOptions: { ecmaVersion: 'latest', sourceType: 'module', ecmaFeatures: { jsx: true } },
  plugins: ['@typescript-eslint', 'react-hooks'],
  extends: ['eslint:recommended', 'plugin:@typescript-eslint/recommended', 'plugin:react-hooks/recommended'],
  rules: {
    '@typescript-eslint/no-unused-vars': ['error', { argsIgnorePattern: '^_', varsIgnorePattern: '^_' }],
    'no-extra-semi': 'off',
    'react-hooks/exhaustive-deps': 'off',
    // The @mui/material barrel loads every MUI component (~7 s per vitest worker); import each from its own path.
    'no-restricted-imports': ['error', { paths: [{ name: '@mui/material', message: "Import from '@mui/material/<Component>' (the barrel loads all of MUI)." }] }],
  },
  ignorePatterns: ['dist', 'node_modules'],
}
