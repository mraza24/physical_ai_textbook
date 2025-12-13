import claude, { CLAUDE_MODEL, DEFAULT_MAX_TOKENS } from '../config/claude';
import cacheService from './cache.service';

const CACHE_TTL_SECONDS = 7 * 24 * 60 * 60; // 7 days

export interface TranslationResult {
  translatedContent: string;
  cacheHit: boolean;
  technicalTerms: Record<string, string>;
}

export class TranslationService {
  /**
   * Translate chapter content to target language
   * Caches result for 7 days
   */
  async translateChapter(
    chapterId: string,
    originalContent: string,
    targetLanguage: string = 'urdu'
  ): Promise<TranslationResult> {
    // Check cache first
    const cacheKey = `translate:${chapterId}:${targetLanguage}`;
    const cachedData = await cacheService.get(cacheKey);

    if (cachedData) {
      try {
        const parsed = JSON.parse(cachedData);
        return {
          translatedContent: parsed.content,
          cacheHit: true,
          technicalTerms: parsed.terms || {},
        };
      } catch (error) {
        console.error('Failed to parse cached translation:', error);
        // Continue to generate new translation
      }
    }

    // Generate translated content with Claude
    const { translatedContent, technicalTerms } = await this.generateTranslation(
      originalContent,
      targetLanguage
    );

    // Cache for 7 days
    const cacheData = JSON.stringify({
      content: translatedContent,
      terms: technicalTerms,
    });
    await cacheService.set(cacheKey, cacheData, CACHE_TTL_SECONDS);

    return {
      translatedContent,
      cacheHit: false,
      technicalTerms,
    };
  }

  /**
   * Call Claude API to translate content
   */
  private async generateTranslation(
    originalContent: string,
    targetLanguage: string
  ): Promise<{ translatedContent: string; technicalTerms: Record<string, string> }> {
    const prompt = this.buildTranslationPrompt(originalContent, targetLanguage);

    const response = await claude.messages.create({
      model: CLAUDE_MODEL,
      max_tokens: DEFAULT_MAX_TOKENS,
      messages: [
        {
          role: 'user',
          content: prompt,
        },
      ],
    });

    // Extract text from response
    const textContent = response.content
      .filter((block) => block.type === 'text')
      .map((block: any) => block.text)
      .join('\n');

    // Extract technical terms from response (if present)
    const technicalTerms = this.extractTechnicalTerms(textContent);

    // Remove technical terms section from translated content
    const translatedContent = textContent.split('---TECHNICAL-TERMS---')[0].trim();

    return { translatedContent, technicalTerms };
  }

  /**
   * Build Claude prompt for translation
   */
  private buildTranslationPrompt(originalContent: string, targetLanguage: string): string {
    const languageInstructions = this.getLanguageInstructions(targetLanguage);

    return `You are an expert technical translator specializing in robotics and AI textbooks.

**Target Language**: ${targetLanguage === 'urdu' ? 'Urdu (اردو)' : targetLanguage}

**Translation Instructions**:
${languageInstructions}

**CRITICAL PRESERVATION RULES**:
1. **Code blocks** (between \`\`\` markers): Keep 100% in English, including comments
2. **Inline code** (between \` markers): Keep in English
3. **Technical terms**: Transliterate but add original in parentheses first time
   - Examples: "ROS 2" → "آر او ایس 2 (ROS 2)"
   - "Node" → "نوڈ (Node)"
   - "Publisher" → "پبلشر (Publisher)"
4. **Markdown structure**: Preserve all #, -, *, >, etc.
5. **Mermaid diagrams**: Translate labels but preserve syntax
6. **URLs and file paths**: Keep in English

**Original Chapter Content**:
${originalContent}

**Task**: Translate the content following the rules above. After the translation, add a section:
---TECHNICAL-TERMS---
{JSON object mapping English technical terms to ${targetLanguage} transliterations}

Return the translated content followed by the technical terms section.`;
  }

  /**
   * Get language-specific translation instructions
   */
  private getLanguageInstructions(targetLanguage: string): string {
    if (targetLanguage === 'urdu') {
      return `- Translate all paragraphs, headings, lists, and explanations to Urdu
- Use clear, educational Urdu suitable for university-level students
- Transliterate English technical terms using Urdu script (e.g., "Python" → "پائتھن")
- Maintain right-to-left (RTL) text flow for Urdu text
- Use proper Urdu punctuation (۔ for period, ؟ for question mark)
- Keep mathematical symbols and numbers in Western format (0-9)
- Technical acronyms: transliterate + show English in parentheses first time`;
    }

    // Default for other languages
    return `- Translate all paragraphs, headings, lists, and explanations to ${targetLanguage}
- Use clear, educational language suitable for university-level students
- Transliterate English technical terms appropriately
- Preserve technical accuracy and meaning`;
  }

  /**
   * Extract technical terms mapping from Claude response
   */
  private extractTechnicalTerms(response: string): Record<string, string> {
    try {
      const match = response.match(/---TECHNICAL-TERMS---\s*(\{[\s\S]*?\})/);
      if (match && match[1]) {
        return JSON.parse(match[1]);
      }
    } catch (error) {
      console.error('Failed to extract technical terms:', error);
    }

    // Return empty object if extraction fails
    return {};
  }

  /**
   * Translate Mermaid diagram labels (helper method)
   * This is a simple regex-based approach for basic label translation
   */
  private translateMermaidLabels(
    mermaidCode: string,
    labelTranslations: Record<string, string>
  ): string {
    let translated = mermaidCode;

    // Translate labels in brackets: [Label Text]
    for (const [english, translated_text] of Object.entries(labelTranslations)) {
      const regex = new RegExp(`\\[${english}\\]`, 'g');
      translated = translated.replace(regex, `[${translated_text}]`);
    }

    return translated;
  }
}

// Export singleton instance
export default new TranslationService();
