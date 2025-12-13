import claude, { CLAUDE_MODEL, DEFAULT_MAX_TOKENS } from '../config/claude';
import cacheService from './cache.service';
import { ProfileResponse } from '../models/user-profile';
import { getPersonalizationCacheKey } from '../utils/profile-hash';

const CACHE_TTL_SECONDS = 24 * 60 * 60; // 24 hours

export interface PersonalizationResult {
  personalizedContent: string;
  cacheHit: boolean;
  profile: ProfileResponse;
}

export class PersonalizationService {
  /**
   * Personalize chapter content based on user profile
   * Caches result for 24 hours using profile hash
   */
  async personalizeChapter(
    chapterId: string,
    originalContent: string,
    profile: ProfileResponse
  ): Promise<PersonalizationResult> {
    // Check cache first
    const cacheKey = getPersonalizationCacheKey(chapterId, profile);
    const cachedContent = await cacheService.get(cacheKey);

    if (cachedContent) {
      return {
        personalizedContent: cachedContent,
        cacheHit: true,
        profile,
      };
    }

    // Generate personalized content with Claude
    const personalizedContent = await this.generatePersonalizedContent(
      originalContent,
      profile
    );

    // Cache for 24 hours
    await cacheService.set(cacheKey, personalizedContent, CACHE_TTL_SECONDS);

    return {
      personalizedContent,
      cacheHit: false,
      profile,
    };
  }

  /**
   * Call Claude API to adapt content based on user profile
   */
  private async generatePersonalizedContent(
    originalContent: string,
    profile: ProfileResponse
  ): Promise<string> {
    const prompt = this.buildPrompt(originalContent, profile);

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

    return textContent.trim();
  }

  /**
   * Build Claude prompt with personalization instructions
   * Adapts verbosity, concept depth, and hardware recommendations
   */
  private buildPrompt(originalContent: string, profile: ProfileResponse): string {
    const instructions = this.generateInstructions(profile);

    return `You are an expert technical editor adapting robotics textbook content to match a student's background.

**Student Profile**:
- Python Experience: ${profile.python_level}
- ROS 2 Experience: ${profile.ros2_level}
- GPU Access: ${profile.gpu_available}
- Hardware Tier: ${profile.hardware_tier}
- Primary Goal: ${profile.primary_goal}

**Personalization Instructions**:
${instructions}

**Original Chapter Content**:
${originalContent}

**Task**: Rewrite the chapter content following the personalization instructions above. Preserve all code examples, headings, and structure exactly as they are. Only modify the explanatory prose to match the student's background level. Return ONLY the personalized chapter content without any meta-commentary.`;
  }

  /**
   * Generate specific instructions based on user profile
   */
  private generateInstructions(profile: ProfileResponse): string {
    const instructions: string[] = [];

    // Python level adjustments
    if (profile.python_level === 'Beginner') {
      instructions.push(
        '- Add brief explanations of Python concepts (classes, decorators, type hints) when they appear'
      );
      instructions.push('- Expand on Python syntax in code examples with inline comments');
    } else if (profile.python_level === 'Expert') {
      instructions.push('- Minimize Python syntax explanations - assume deep Python knowledge');
      instructions.push('- Focus on ROS 2-specific patterns rather than general Python concepts');
    }

    // ROS 2 level adjustments
    if (profile.ros2_level === 'None') {
      instructions.push(
        '- Expand ROS 2 concepts significantly - this is the student\'s first exposure'
      );
      instructions.push('- Add context for terms like "node", "topic", "QoS", "executor"');
      instructions.push('- Include motivation for WHY each ROS 2 pattern exists');
    } else if (profile.ros2_level === 'Intermediate' || profile.ros2_level === 'Expert') {
      instructions.push('- Reduce basic ROS 2 explanations - student already knows fundamentals');
      instructions.push('- Focus on advanced patterns, performance tips, and edge cases');
    }

    // GPU availability adjustments
    if (profile.gpu_available === 'No') {
      instructions.push('- Emphasize CPU-only alternatives and lightweight models');
      instructions.push('- Mention that GPU sections are optional for understanding concepts');
      instructions.push('- Suggest cloud GPU options (Colab, AWS, GCP) when GPU is required');
    } else if (profile.gpu_available === 'Yes') {
      instructions.push('- Include GPU-accelerated workflows prominently');
      instructions.push('- Add performance comparisons (CPU vs GPU) where relevant');
    } else if (profile.gpu_available === 'Cloud') {
      instructions.push('- Mention cloud GPU setup (Colab Pro, AWS g4dn, GCP T4) when relevant');
      instructions.push('- Include cost estimates for cloud GPU training sessions');
    }

    // Hardware tier adjustments
    if (profile.hardware_tier === 'Tier1') {
      instructions.push(
        '- Emphasize edge device workflows (Jetson Nano, Raspberry Pi 4, lightweight models)'
      );
      instructions.push('- Suggest model quantization and optimization techniques');
      instructions.push('- Note which examples require more powerful hardware');
    } else if (profile.hardware_tier === 'Tier2') {
      instructions.push(
        '- Balance between edge and desktop workflows (Jetson Orin Nano, RTX 3060)'
      );
      instructions.push('- Mention when Tier 3 hardware would significantly improve performance');
    } else if (profile.hardware_tier === 'Tier3') {
      instructions.push(
        '- Highlight advanced features available with powerful hardware (Jetson AGX, RTX 4070+)'
      );
      instructions.push('- Include multi-GPU setups and large model training where applicable');
    }

    // Primary goal adjustments
    if (profile.primary_goal === 'Learning') {
      instructions.push('- Add learning tips, common mistakes, and debugging strategies');
      instructions.push('- Suggest hands-on exercises to reinforce concepts');
    } else if (profile.primary_goal === 'Research') {
      instructions.push('- Add references to recent papers and cutting-edge techniques');
      instructions.push('- Mention research opportunities and open problems');
    } else if (profile.primary_goal === 'Teaching') {
      instructions.push('- Add pedagogical notes for instructors');
      instructions.push('- Suggest assessment questions and grading rubrics');
    }

    return instructions.join('\n');
  }
}

// Export singleton instance
export default new PersonalizationService();
