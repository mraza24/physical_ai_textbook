/**
 * GHOST DIV DETECTOR
 * Automatically finds and reports invisible blocking elements
 * Load this in browser console or add to <head> for debugging
 */

(function() {
  'use strict';

  console.log('🔍 Ghost Div Detector - Starting scan...');

  // Find all elements
  const allElements = document.querySelectorAll('*');
  const ghostDivs = [];

  allElements.forEach(el => {
    const computed = window.getComputedStyle(el);
    const rect = el.getBoundingClientRect();

    // Criteria for "ghost div":
    // 1. Fixed or absolute positioning
    // 2. Takes up significant space (width > 100px OR height > 100px)
    // 3. Either transparent background OR no background
    // 4. Has pointer-events: auto (can block clicks)
    // 5. High z-index (> 100)

    const isPositioned = computed.position === 'fixed' || computed.position === 'absolute';
    const isLarge = rect.width > 100 || rect.height > 100;
    const canBlock = computed.pointerEvents !== 'none';
    const hasHighZIndex = parseInt(computed.zIndex) > 100 || computed.zIndex === 'auto';

    if (isPositioned && isLarge && canBlock) {
      ghostDivs.push({
        element: el,
        tag: el.tagName,
        id: el.id,
        className: el.className,
        position: computed.position,
        width: rect.width,
        height: rect.height,
        top: rect.top,
        left: rect.left,
        right: rect.right,
        bottom: rect.bottom,
        zIndex: computed.zIndex,
        pointerEvents: computed.pointerEvents,
        background: computed.background,
        opacity: computed.opacity
      });
    }
  });

  console.log(`\n🎯 Found ${ghostDivs.length} potential ghost divs:\n`);

  // Sort by z-index (highest first)
  ghostDivs.sort((a, b) => {
    const aZ = parseInt(a.zIndex) || 0;
    const bZ = parseInt(b.zIndex) || 0;
    return bZ - aZ;
  });

  // Report findings
  ghostDivs.forEach((ghost, index) => {
    console.group(`Ghost Div #${index + 1}: ${ghost.tag}${ghost.id ? '#' + ghost.id : ''}${ghost.className ? '.' + ghost.className.split(' ')[0] : ''}`);
    console.log('Element:', ghost.element);
    console.log('Position:', ghost.position);
    console.log('Dimensions:', `${Math.round(ghost.width)}px × ${Math.round(ghost.height)}px`);
    console.log('Location:', `top: ${Math.round(ghost.top)}px, left: ${Math.round(ghost.left)}px`);
    console.log('Z-Index:', ghost.zIndex);
    console.log('Pointer Events:', ghost.pointerEvents);
    console.log('Opacity:', ghost.opacity);
    console.groupEnd();
  });

  // Check navbar specifically
  console.log('\n📊 Navbar Analysis:');
  const navbar = document.querySelector('nav.navbar');
  if (navbar) {
    const navComputed = window.getComputedStyle(navbar);
    const navRect = navbar.getBoundingClientRect();
    console.log('Navbar z-index:', navComputed.zIndex);
    console.log('Navbar position:', navComputed.position);
    console.log('Navbar pointer-events:', navComputed.pointerEvents);
    console.log('Navbar dimensions:', `${Math.round(navRect.width)}px × ${Math.round(navRect.height)}px`);
    console.log('Navbar border:', navComputed.border);
  } else {
    console.warn('⚠️ Navbar not found!');
  }

  // Check Bulldog position
  console.log('\n🐕 Bulldog Assistant Analysis:');
  const bulldog = document.querySelector('[class*="bulldog"]');
  if (bulldog) {
    const bulldogComputed = window.getComputedStyle(bulldog);
    const bulldogRect = bulldog.getBoundingClientRect();
    console.log('Bulldog element:', bulldog);
    console.log('Bulldog z-index:', bulldogComputed.zIndex);
    console.log('Bulldog position:', bulldogComputed.position);
    console.log('Bulldog pointer-events:', bulldogComputed.pointerEvents);
    console.log('Bulldog right edge:', Math.round(bulldogRect.right) + 'px');
    console.log('Window width:', window.innerWidth + 'px');
    console.log('Distance from right:', Math.round(window.innerWidth - bulldogRect.right) + 'px (should be ~20px)');

    if (Math.abs(window.innerWidth - bulldogRect.right - 20) > 5) {
      console.error('🚨 GHOST DIV CONFIRMED: Bulldog is NOT at right edge! It\'s shifted LEFT by a blocking element.');
    } else {
      console.log('✅ Bulldog position is correct');
    }
  } else {
    console.warn('⚠️ Bulldog not found!');
  }

  // Check buttons
  console.log('\n🔘 Button Click Test:');
  const buttons = document.querySelectorAll('a[href*="login"], a[href*="signup"], .primaryButton, .secondaryButton, button');
  console.log(`Found ${buttons.length} buttons/links`);

  buttons.forEach((btn, i) => {
    const btnComputed = window.getComputedStyle(btn);
    const btnRect = btn.getBoundingClientRect();
    const isClickable = btnComputed.pointerEvents !== 'none' && btnComputed.cursor === 'pointer';

    console.log(`Button ${i + 1}: ${btn.textContent?.trim() || btn.getAttribute('href') || 'Unknown'}`,
      isClickable ? '✅ Clickable' : '❌ BLOCKED',
      `(z-index: ${btnComputed.zIndex}, pointer-events: ${btnComputed.pointerEvents})`
    );
  });

  // Highlight ghost divs visually
  console.log('\n🎨 Highlighting ghost divs in yellow...');
  ghostDivs.forEach((ghost, index) => {
    ghost.element.style.outline = '3px solid yellow';
    ghost.element.style.outlineOffset = '-3px';

    // Add label
    const label = document.createElement('div');
    label.textContent = `Ghost #${index + 1}`;
    label.style.cssText = `
      position: fixed;
      top: ${ghost.top}px;
      left: ${ghost.left}px;
      background: yellow;
      color: black;
      padding: 4px 8px;
      font-size: 12px;
      font-weight: bold;
      z-index: 999999;
      pointer-events: none;
    `;
    document.body.appendChild(label);
  });

  console.log('\n✅ Ghost Div Detector - Scan complete!');
  console.log('💡 Check the console output and yellow outlines to identify blocking elements.');

  return ghostDivs;
})();
