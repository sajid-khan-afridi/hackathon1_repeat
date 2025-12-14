#!/usr/bin/env node

/**
 * Urdu Translator Skill - JavaScript Entry Point
 *
 * This is a wrapper that invokes the Python implementation.
 * The core translation logic is in tools/translate_to_urdu.py
 */

import { spawn } from 'child_process';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

/**
 * Translate content to Urdu using the Python implementation
 */
async function translateToUrdu(options = {}) {
  const {
    content,
    inputFile,
    outputFile,
    preserveTerms = []
  } = options;

  return new Promise((resolve, reject) => {
    const pythonScript = path.join(__dirname, 'tools', 'translate_to_urdu.py');
    const args = [];

    if (content) {
      args.push('--text', content);
    } else if (inputFile) {
      args.push(inputFile);
      if (outputFile) {
        args.push(outputFile);
      }
    } else {
      reject(new Error('Either content or inputFile is required'));
      return;
    }

    const python = spawn('python', [pythonScript, ...args], {
      env: {
        ...process.env,
        PYTHONIOENCODING: 'utf-8'
      }
    });

    let stdout = '';
    let stderr = '';

    python.stdout.on('data', (data) => {
      stdout += data.toString();
    });

    python.stderr.on('data', (data) => {
      stderr += data.toString();
    });

    python.on('close', (code) => {
      if (code === 0) {
        try {
          const result = JSON.parse(stdout);
          resolve(result);
        } catch (e) {
          // Not JSON output (file was written)
          resolve({
            success: true,
            message: stdout.trim(),
            outputFile
          });
        }
      } else {
        reject(new Error(stderr || `Process exited with code ${code}`));
      }
    });

    python.on('error', (err) => {
      reject(new Error(`Failed to start Python: ${err.message}`));
    });
  });
}

/**
 * CLI interface
 */
async function main() {
  const args = process.argv.slice(2);

  if (args.length < 1) {
    console.error('Usage: urdu-translator <operation> [options]');
    console.error('Operations: translate');
    console.error('');
    console.error('Examples:');
    console.error('  urdu-translator translate --input file.mdx --output file.ur.mdx');
    console.error('  urdu-translator translate --text "Hello world"');
    process.exit(1);
  }

  const operation = args[0];
  const options = parseCliArgs(args.slice(1));

  try {
    let result;
    switch (operation) {
      case 'translate':
        result = await translateToUrdu({
          content: options.text,
          inputFile: options.input,
          outputFile: options.output,
          preserveTerms: options.preserve ? options.preserve.split(',') : []
        });
        break;

      default:
        console.error('Unknown operation:', operation);
        process.exit(1);
    }

    console.log(JSON.stringify(result, null, 2));
  } catch (error) {
    console.error('Error:', error.message);
    process.exit(1);
  }
}

/**
 * Parse CLI arguments
 */
function parseCliArgs(args) {
  const options = {};
  for (let i = 0; i < args.length; i++) {
    if (args[i].startsWith('--')) {
      const key = args[i].slice(2);
      const value = args[i + 1] && !args[i + 1].startsWith('--') ? args[i + 1] : 'true';
      options[key] = value;
      if (value !== 'true') i++;
    }
  }
  return options;
}

// Export for use as a module
export default translateToUrdu;
export { translateToUrdu };

// Run CLI if called directly
const isMainModule = import.meta.url === `file://${process.argv[1]}` ||
                     process.argv[1]?.endsWith('urdu-translator/index.js');
if (isMainModule) {
  main().catch(console.error);
}
