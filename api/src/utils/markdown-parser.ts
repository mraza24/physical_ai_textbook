import { unified } from 'unified';
import remarkParse from 'remark-parse';
import remarkStringify from 'remark-stringify';
import remarkMdx from 'remark-mdx';

/**
 * Parse MDX to AST, extract text for personalization
 * Excludes code blocks to preserve technical examples
 */
export async function parseMdx(content: string): Promise<{ ast: any; textContent: string }> {
  const processor = unified()
    .use(remarkParse)
    .use(remarkMdx);

  const ast = processor.parse(content);
  const textContent = extractTextFromAst(ast);

  return { ast, textContent };
}

/**
 * Reconstruct MDX from modified AST
 */
export async function stringifyMdx(ast: any): Promise<string> {
  const processor = unified()
    .use(remarkStringify, {
      bullet: '-',
      emphasis: '_',
      strong: '*',
      fences: true,
    })
    .use(remarkMdx);

  const result = processor.stringify(ast);
  return result;
}

/**
 * Extract text content from AST, excluding code blocks
 * Used to send only prose to Claude for personalization
 */
function extractTextFromAst(node: any): string {
  const textParts: string[] = [];

  function traverse(n: any) {
    // Skip code blocks and inline code
    if (n.type === 'code' || n.type === 'inlineCode') {
      return;
    }

    // Extract text nodes
    if (n.type === 'text') {
      textParts.push(n.value);
    }

    // Recursively traverse children
    if (n.children && Array.isArray(n.children)) {
      n.children.forEach((child: any) => traverse(child));
    }
  }

  traverse(node);
  return textParts.join(' ').trim();
}

/**
 * Simple text replacement in MDX (preserves code blocks)
 * Used when Claude returns full chapter text without structure
 */
export function replaceTextInMdx(
  originalMdx: string,
  replacements: Array<{ original: string; replacement: string }>
): string {
  let result = originalMdx;

  // Apply replacements outside code blocks
  const codeBlockRegex = /```[\s\S]*?```|`[^`]*`/g;
  const codeBlocks: string[] = [];
  let match;
  let index = 0;

  // Extract code blocks
  while ((match = codeBlockRegex.exec(originalMdx)) !== null) {
    codeBlocks.push(match[0]);
  }

  // Replace code blocks with placeholders
  result = result.replace(codeBlockRegex, () => `__CODE_BLOCK_${index++}__`);

  // Apply text replacements
  for (const { original, replacement } of replacements) {
    result = result.replace(new RegExp(escapeRegex(original), 'g'), replacement);
  }

  // Restore code blocks
  index = 0;
  result = result.replace(/__CODE_BLOCK_(\d+)__/g, () => codeBlocks[index++] || '');

  return result;
}

/**
 * Escape special regex characters
 */
function escapeRegex(str: string): string {
  return str.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}
