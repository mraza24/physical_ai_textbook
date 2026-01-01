import { unified } from 'unified';
import remarkParse from 'remark-parse';
import remarkStringify from 'remark-stringify';
import type { Root, Code, InlineCode } from 'mdast';

/**
 * Markdown Processor Service
 *
 * Uses remark + unified for AST-based markdown processing.
 *
 * Key Features:
 * - Preserves Docusaurus-specific syntax (admonitions, imports, tabs)
 * - Extracts code blocks for protection during LLM transformation
 * - Validates markdown structure
 * - Detects Docusaurus components
 *
 * Critical for ensuring AI transformations don't break the textbook!
 */

/**
 * Docusaurus-specific patterns to preserve
 */
const DOCUSAURUS_PATTERNS = {
  // Admonitions: :::note, :::tip, :::warning, :::danger, :::info
  admonition: /^:::(note|tip|warning|danger|info|caution)/gm,

  // Import statements: import Component from '@site/...'
  import: /^import\s+.+from\s+['"]@site\/.+['"]/gm,

  // MDX components: <Tabs>, <TabItem>, etc.
  component: /<[A-Z][a-zA-Z0-9]*(\s+[^>]*)?(>|\/?>)/g,

  // Code blocks with language tags
  codeBlock: /```[\w-]+\n[\s\S]*?\n```/g,

  // Frontmatter (YAML between ---)
  frontmatter: /^---\n[\s\S]*?\n---/m,
};

/**
 * Extract code blocks from markdown
 *
 * Replaces code blocks with placeholders to protect them during transformation
 *
 * @param markdown - Original markdown content
 * @returns Object with processed markdown and code blocks map
 */
export function extractCodeBlocks(markdown: string): {
  processedMarkdown: string;
  codeBlocks: Map<string, string>;
} {
  const codeBlocks = new Map<string, string>();
  let blockIndex = 0;

  const processedMarkdown = markdown.replace(
    /```([\w-]*)\n([\s\S]*?)\n```/g,
    (match, language, code) => {
      const placeholder = `CODE_BLOCK_${blockIndex}`;
      codeBlocks.set(placeholder, match);
      blockIndex++;
      return `\n${placeholder}\n`;
    }
  );

  return { processedMarkdown, codeBlocks };
}

/**
 * Restore code blocks from placeholders
 *
 * Replaces placeholders with original code blocks
 *
 * @param markdown - Markdown with placeholders
 * @param codeBlocks - Map of placeholder → original code block
 * @returns Markdown with restored code blocks
 */
export function restoreCodeBlocks(
  markdown: string,
  codeBlocks: Map<string, string>
): string {
  let restored = markdown;

  for (const [placeholder, codeBlock] of codeBlocks.entries()) {
    restored = restored.replace(new RegExp(placeholder, 'g'), codeBlock);
  }

  return restored;
}

/**
 * Extract Docusaurus components from markdown
 *
 * Identifies import statements, admonitions, and MDX components
 *
 * @param markdown - Markdown content
 * @returns Array of component matches
 */
export function extractDocusaurusComponents(markdown: string): {
  imports: string[];
  admonitions: string[];
  components: string[];
} {
  const imports: string[] = [];
  const admonitions: string[] = [];
  const components: string[] = [];

  // Extract imports
  let match;
  while ((match = DOCUSAURUS_PATTERNS.import.exec(markdown)) !== null) {
    imports.push(match[0]);
  }

  // Extract admonitions
  const admonitionRegex = /^:::(note|tip|warning|danger|info|caution).*?\n([\s\S]*?)^:::/gm;
  while ((match = admonitionRegex.exec(markdown)) !== null) {
    admonitions.push(match[0]);
  }

  // Extract MDX components
  while ((match = DOCUSAURUS_PATTERNS.component.exec(markdown)) !== null) {
    components.push(match[0]);
  }

  return { imports, admonitions, components };
}

/**
 * Validate markdown structure
 *
 * Checks for common issues that could break Docusaurus build
 *
 * @param markdown - Markdown content
 * @returns Validation result with warnings
 */
export function validateMarkdown(markdown: string): {
  valid: boolean;
  warnings: string[];
} {
  const warnings: string[] = [];

  // Check for unclosed admonitions
  const admonitionStarts = (markdown.match(/^:::/gm) || []).length;
  if (admonitionStarts % 2 !== 0) {
    warnings.push('Unclosed admonition detected (odd number of ::: markers)');
  }

  // Check for unclosed code blocks
  const codeBlockMarkers = (markdown.match(/```/g) || []).length;
  if (codeBlockMarkers % 2 !== 0) {
    warnings.push('Unclosed code block detected (odd number of ``` markers)');
  }

  // Check for broken import statements
  const imports = markdown.match(DOCUSAURUS_PATTERNS.import) || [];
  imports.forEach(importStatement => {
    if (!importStatement.includes('@site/')) {
      warnings.push(`Non-standard import detected: ${importStatement}`);
    }
  });

  // Check for broken MDX components
  const components = markdown.match(/<[A-Z][a-zA-Z0-9]*/g) || [];
  const closingTags = markdown.match(/<\/[A-Z][a-zA-Z0-9]*>/g) || [];
  const selfClosingTags = markdown.match(/<[A-Z][a-zA-Z0-9]*[^>]*\/>/g) || [];

  const expectedClosing = components.length - (selfClosingTags?.length || 0);
  if (closingTags.length !== expectedClosing) {
    warnings.push(`Possible unclosed MDX component (${components.length} opening, ${closingTags.length} closing)`);
  }

  return {
    valid: warnings.length === 0,
    warnings,
  };
}

/**
 * Parse markdown to AST
 *
 * Uses remark parser for structured access to markdown elements
 *
 * @param markdown - Markdown content
 * @returns Markdown AST (Abstract Syntax Tree)
 */
export async function parseMarkdown(markdown: string): Promise<Root> {
  const processor = unified().use(remarkParse);
  const ast = processor.parse(markdown);
  return ast as Root;
}

/**
 * Stringify AST back to markdown
 *
 * Converts markdown AST to string with proper formatting
 *
 * @param ast - Markdown AST
 * @returns Markdown string
 */
export async function stringifyMarkdown(ast: Root): Promise<string> {
  const processor = unified().use(remarkStringify, {
    bullet: '-',
    fence: '`',
    fences: true,
    incrementListMarker: true,
  });

  const result = processor.stringify(ast);
  return result;
}

/**
 * Count words in markdown (excluding code blocks)
 *
 * Useful for estimating LLM token usage
 *
 * @param markdown - Markdown content
 * @returns Word count
 */
export function countWords(markdown: string): number {
  // Remove code blocks
  const withoutCode = markdown.replace(/```[\s\S]*?```/g, '');

  // Remove inline code
  const withoutInlineCode = withoutCode.replace(/`[^`]+`/g, '');

  // Remove markdown syntax
  const plainText = withoutInlineCode
    .replace(/[#*_~`\[\]()]/g, '') // Remove markdown chars
    .replace(/https?:\/\/[^\s]+/g, '') // Remove URLs
    .replace(/\n+/g, ' ') // Normalize whitespace
    .trim();

  // Count words
  const words = plainText.split(/\s+/).filter(word => word.length > 0);
  return words.length;
}

/**
 * Estimate LLM token count
 *
 * Rough approximation: 1 token ≈ 0.75 words
 *
 * @param markdown - Markdown content
 * @returns Estimated token count
 */
export function estimateTokens(markdown: string): number {
  const wordCount = countWords(markdown);
  return Math.ceil(wordCount / 0.75);
}

/**
 * Extract frontmatter from markdown
 *
 * Parses YAML frontmatter (between ---) if present
 *
 * @param markdown - Markdown content
 * @returns Object with frontmatter and content
 */
export function extractFrontmatter(markdown: string): {
  frontmatter: Record<string, any> | null;
  content: string;
} {
  const match = markdown.match(/^---\n([\s\S]*?)\n---\n([\s\S]*)$/);

  if (!match) {
    return {
      frontmatter: null,
      content: markdown,
    };
  }

  const frontmatterText = match[1];
  const content = match[2];

  // Simple YAML parsing (key: value pairs)
  const frontmatter: Record<string, any> = {};
  frontmatterText.split('\n').forEach(line => {
    const colonIndex = line.indexOf(':');
    if (colonIndex > 0) {
      const key = line.substring(0, colonIndex).trim();
      const value = line.substring(colonIndex + 1).trim();
      frontmatter[key] = value.replace(/^['"]|['"]$/g, ''); // Remove quotes
    }
  });

  return {
    frontmatter,
    content,
  };
}

/**
 * Full transformation pipeline
 *
 * Safe processing with code block protection and validation
 *
 * @param markdown - Original markdown
 * @param transformFn - Async function that transforms markdown
 * @returns Transformed markdown with preserved code blocks
 */
export async function safeTransform(
  markdown: string,
  transformFn: (text: string) => Promise<string>
): Promise<{ transformed: string; warnings: string[] }> {
  // Extract code blocks
  const { processedMarkdown, codeBlocks } = extractCodeBlocks(markdown);

  // Extract frontmatter
  const { frontmatter, content } = extractFrontmatter(processedMarkdown);

  // Transform content
  const transformed = await transformFn(content);

  // Restore code blocks
  const withCodeBlocks = restoreCodeBlocks(transformed, codeBlocks);

  // Restore frontmatter if it existed
  const final = frontmatter
    ? `---\n${Object.entries(frontmatter).map(([k, v]) => `${k}: ${v}`).join('\n')}\n---\n\n${withCodeBlocks}`
    : withCodeBlocks;

  // Validate result
  const validation = validateMarkdown(final);

  return {
    transformed: final,
    warnings: validation.warnings,
  };
}
