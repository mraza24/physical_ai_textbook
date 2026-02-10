# Research Validator Skill

**Version**: 1.0.0
**Purpose**: Validate translated content to ensure technical terms were not accidentally translated
**Priority**: P4 (Internal Quality Tool)

---

## Overview

The Research Validator skill checks AI-generated translations (especially Urdu translations) to ensure that critical technical terms remain in English as specified in the technical glossary. This prevents confusion and maintains consistency with industry standards.

---

## Process

### 1. Input Validation

**Required Inputs**:
- `original_content`: Original English markdown content
- `translated_content`: Translated Urdu markdown content
- `technical_glossary`: List of terms that should NOT be translated

**Validation Checks**:
- Both original and translated content must be non-empty
- Technical glossary must contain at least 1 term
- Content must be valid markdown format

### 2. Technical Term Extraction

**Scan Original Content**:
- Extract all instances of technical terms from glossary
- Record positions and contexts
- Build expected term list

**Expected Behavior**:
- Case-insensitive matching
- Whole-word matching (avoid partial matches)
- Preserve original casing from glossary

### 3. Validation Analysis

**Check Translated Content**:
- Verify each technical term appears in translated content
- Confirm terms are in English (not transliterated or translated)
- Check frequency matches (original vs. translated)

**Violation Detection**:
- Missing terms (term in original but not in translation)
- Translated terms (term appears in Urdu instead of English)
- Extra terms (term appears more times in translation than original)

### 4. Generate Report

**Validation Report Structure**:
```json
{
  "status": "PASS" | "FAIL",
  "total_terms_checked": 45,
  "violations": [
    {
      "term": "PID Controller",
      "issue": "translated",
      "expected": "PID Controller",
      "found": "پی آئی ڈی کنٹرولر",
      "line": 23,
      "severity": "high"
    }
  ],
  "preserved_correctly": [
    "ROS2", "SLAM", "LIDAR", ...
  ],
  "suggestions": [
    "Review line 23: 'PID Controller' should remain in English",
    "Ensure translator preserves technical glossary terms"
  ]
}
```

### 5. Quality Criteria

**Pass Criteria**:
- ✅ ALL technical terms preserved in English
- ✅ Term frequency matches original content (±10% acceptable)
- ✅ No transliteration of technical terms
- ✅ Code blocks unchanged from original

**Fail Criteria**:
- ❌ ANY technical term translated to Urdu
- ❌ Missing technical terms (> 10% of original occurrences)
- ❌ Code blocks modified or translated

---

## Examples

### Example 1: PASS - Correct Preservation

**Original**:
```markdown
# Introduction to ROS2

ROS2 is a robotics framework. It uses a PID Controller for motor control.
SLAM helps with Localization and Mapping.
```

**Translated**:
```markdown
# ROS2 کا تعارف

ROS2 ایک روبوٹکس فریم ورک ہے۔ یہ موٹر کنٹرول کے لیے PID Controller استعمال کرتا ہے۔
SLAM Localization اور Mapping میں مدد کرتا ہے۔
```

**Validation**: ✅ PASS
- ROS2: ✅ Preserved
- PID Controller: ✅ Preserved
- SLAM: ✅ Preserved
- Localization: ✅ Preserved
- Mapping: ✅ Preserved

### Example 2: FAIL - Translated Term

**Original**:
```markdown
LIDAR sensors provide accurate distance measurements.
```

**Translated**:
```markdown
لائیڈار سینسرز درست فاصلے کی پیمائش فراہم کرتے ہیں۔
```

**Validation**: ❌ FAIL
- LIDAR: ❌ Transliterated to "لائیڈار" instead of preserved as "LIDAR"

**Report**:
```json
{
  "status": "FAIL",
  "violations": [
    {
      "term": "LIDAR",
      "issue": "transliterated",
      "expected": "LIDAR",
      "found": "لائیڈار",
      "severity": "high",
      "suggestion": "Keep 'LIDAR' in English as per technical glossary"
    }
  ]
}
```

### Example 3: PASS - Code Block Preservation

**Original**:
```markdown
Install ROS2:
\`\`\`bash
sudo apt install ros-humble-desktop
\`\`\`
```

**Translated**:
```markdown
ROS2 انسٹال کریں:
\`\`\`bash
sudo apt install ros-humble-desktop
\`\`\`
```

**Validation**: ✅ PASS
- ROS2: ✅ Preserved
- Code block: ✅ Unchanged

---

## Integration

### Backend API Usage

```typescript
// backend/src/services/research-validator.ts
import { validateTranslation } from '.claude/skills/research-validator';

const validationReport = await validateTranslation({
  original_content: originalMarkdown,
  translated_content: translatedMarkdown,
  technical_glossary: DEFAULT_TECHNICAL_TERMS, // From translate.ts
});

if (validationReport.status === 'FAIL') {
  console.warn('Translation validation failed:', validationReport.violations);
  // Optionally return validation report to frontend
}
```

### Frontend Display

```typescript
// Show validation status to users
if (translationMetadata.validation_status === 'FAIL') {
  <div className="validation-warning">
    ⚠️ Some technical terms may have been translated.
    <a href="#">View Report</a>
  </div>
}
```

---

## Error Handling

**Common Issues**:
1. **Malformed Markdown**: Return validation error, suggest markdown linting
2. **Empty Glossary**: Use DEFAULT_TECHNICAL_TERMS as fallback
3. **Missing Content**: Return 400 error with clear message
4. **LLM Timeout**: Retry once, then fail gracefully

**Fallback Behavior**:
- If validation service fails, still return translated content
- Log validation failure for monitoring
- Display generic warning to user

---

## Success Criteria (SC-004)

Per spec.md SC-004: **100% technical term accuracy**

**Measurement**:
```
Accuracy = (Correctly Preserved Terms / Total Technical Terms) × 100%
```

**Target**: 100% (zero tolerance for technical term translation)

**Monitoring**:
- Track validation failures per translation request
- Alert if failure rate > 5%
- Review glossary updates monthly

---

## Future Enhancements

1. **Auto-Correction**: Automatically fix transliterated terms
2. **Context-Aware Validation**: Check term usage in correct context
3. **Multi-Language Support**: Validate Hindi, Arabic, Chinese translations
4. **Batch Validation**: Validate entire textbook at once
5. **LLM-Based Correction**: Use Claude to suggest fixes for violations

---

**Skill Owner**: Physical AI Textbook Team
**Last Updated**: 2026-01-01
**Related Tasks**: T096-T100 (Research Validator implementation)
