"use strict";
/**
 * Research Validator Service
 *
 * Validates translated content to ensure technical terms were not accidentally translated.
 * Implements the research-validator skill from .claude/skills/research-validator/skill.md
 *
 * Success Criteria (SC-004): 100% technical term preservation accuracy
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.validateTranslation = validateTranslation;
exports.validateCodeBlocks = validateCodeBlocks;
/**
 * Validate translated content for technical term preservation
 *
 * @param originalContent - Original English markdown content
 * @param translatedContent - Translated markdown content (e.g., Urdu)
 * @param technicalGlossary - List of terms that should NOT be translated
 * @returns Validation report with status and violations
 */
async function validateTranslation(originalContent, translatedContent, technicalGlossary) {
    const violations = [];
    const preservedCorrectly = [];
    // Normalize content for comparison (remove extra whitespace)
    const normalizedOriginal = originalContent.toLowerCase();
    const normalizedTranslated = translatedContent.toLowerCase();
    // Check each technical term
    for (const term of technicalGlossary) {
        const termLower = term.toLowerCase();
        // Count occurrences in original
        const originalOccurrences = countOccurrences(normalizedOriginal, termLower);
        if (originalOccurrences === 0) {
            // Term not in original content, skip validation
            continue;
        }
        // Count occurrences in translated
        const translatedOccurrences = countOccurrences(normalizedTranslated, termLower);
        // Check if term is preserved
        if (translatedOccurrences === 0) {
            // Term is missing from translation
            violations.push({
                term,
                issue: 'missing',
                expected: term,
                severity: 'high',
                suggestion: `Ensure '${term}' is preserved in English in the translated content`,
            });
        }
        else if (Math.abs(translatedOccurrences - originalOccurrences) > originalOccurrences * 0.1) {
            // Frequency mismatch (> 10% difference)
            violations.push({
                term,
                issue: 'translated',
                expected: term,
                severity: 'medium',
                suggestion: `Check '${term}' frequency: original=${originalOccurrences}, translated=${translatedOccurrences}`,
            });
        }
        else {
            // Term preserved correctly
            preservedCorrectly.push(term);
        }
    }
    // Calculate accuracy
    const totalTermsChecked = technicalGlossary.filter((term) => {
        const termLower = term.toLowerCase();
        return countOccurrences(normalizedOriginal, termLower) > 0;
    }).length;
    const accuracyPercentage = totalTermsChecked > 0
        ? ((preservedCorrectly.length / totalTermsChecked) * 100)
        : 100;
    // Generate suggestions
    const suggestions = [];
    if (violations.length > 0) {
        suggestions.push(`Review ${violations.length} violation(s) and ensure technical terms remain in English`);
        suggestions.push('Refer to the technical glossary in backend/src/routes/translate.ts');
    }
    return {
        status: violations.length === 0 ? 'PASS' : 'FAIL',
        total_terms_checked: totalTermsChecked,
        violations,
        preserved_correctly: preservedCorrectly,
        suggestions,
        accuracy_percentage: accuracyPercentage,
    };
}
/**
 * Count occurrences of a term in content (case-insensitive, whole-word matching)
 */
function countOccurrences(content, term) {
    // Escape special regex characters in term
    const escapedTerm = term.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
    // Match whole words only (word boundaries)
    const regex = new RegExp(`\\b${escapedTerm}\\b`, 'gi');
    const matches = content.match(regex);
    return matches ? matches.length : 0;
}
/**
 * Validate code blocks are unchanged
 *
 * Ensures code blocks in translated content match original content exactly.
 */
function validateCodeBlocks(originalContent, translatedContent) {
    const issues = [];
    // Extract code blocks from both contents
    const originalCodeBlocks = extractCodeBlocks(originalContent);
    const translatedCodeBlocks = extractCodeBlocks(translatedContent);
    // Check count matches
    if (originalCodeBlocks.length !== translatedCodeBlocks.length) {
        issues.push(`Code block count mismatch: original=${originalCodeBlocks.length}, translated=${translatedCodeBlocks.length}`);
        return { valid: false, issues };
    }
    // Check each code block matches exactly
    for (let i = 0; i < originalCodeBlocks.length; i++) {
        if (originalCodeBlocks[i] !== translatedCodeBlocks[i]) {
            issues.push(`Code block ${i + 1} was modified in translation`);
        }
    }
    return {
        valid: issues.length === 0,
        issues,
    };
}
/**
 * Extract code blocks from markdown content
 */
function extractCodeBlocks(content) {
    const codeBlockRegex = /```[\s\S]*?```/g;
    const matches = content.match(codeBlockRegex);
    return matches || [];
}
