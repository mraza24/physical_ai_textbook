/**
 * CLICK FIXER - Runtime Button Unblock
 * Automatically ensures all buttons are clickable on page load
 * This script forcibly fixes any blocking elements
 */

(function() {
  'use strict';

  console.log('🔧 Click Fixer - Starting...');

  function forceButtonClickability() {
    // STEP 1: Force navbar to highest z-index and full clickability
    const navbar = document.querySelector('nav.navbar');
    if (navbar) {
      navbar.style.setProperty('z-index', '9999', 'important');
      navbar.style.setProperty('pointer-events', 'all', 'important');
      navbar.style.setProperty('position', 'relative', 'important');
      console.log('✅ Navbar forced to z-index: 9999');
    }

    // STEP 2: Force all navbar links and buttons clickable
    const navbarElements = document.querySelectorAll('.navbar a, .navbar button, .navbar__link, .navbar__item');
    navbarElements.forEach(el => {
      el.style.setProperty('pointer-events', 'auto', 'important');
      el.style.setProperty('cursor', 'pointer', 'important');
      el.style.setProperty('z-index', '9999', 'important');
    });
    console.log(`✅ Fixed ${navbarElements.length} navbar elements`);

    // STEP 3: Force homepage buttons clickable
    const buttons = document.querySelectorAll('.primaryButton, .secondaryButton, a[href*="login"], a[href*="signup"]');
    buttons.forEach(btn => {
      btn.style.setProperty('pointer-events', 'auto', 'important');
      btn.style.setProperty('cursor', 'pointer', 'important');
      btn.style.setProperty('z-index', '9998', 'important');
      btn.style.setProperty('position', 'relative', 'important');
    });
    console.log(`✅ Fixed ${buttons.length} homepage buttons`);

    // STEP 4: Ensure Bulldog doesn't block
    const bulldogContainer = document.querySelector('[class*="bulldogContainer"]');
    if (bulldogContainer) {
      bulldogContainer.style.setProperty('pointer-events', 'none', 'important');
      bulldogContainer.style.setProperty('position', 'fixed', 'important');
      bulldogContainer.style.setProperty('bottom', '20px', 'important');
      bulldogContainer.style.setProperty('right', '20px', 'important');
      bulldogContainer.style.setProperty('width', '350px', 'important');
      bulldogContainer.style.setProperty('z-index', '1000', 'important');
      console.log('✅ Bulldog container forced to non-blocking');

      // Make FAB and chat window clickable
      const fabButton = bulldogContainer.querySelector('button');
      const chatWindow = bulldogContainer.querySelector('[class*="chatWindow"]');

      if (fabButton) {
        fabButton.style.setProperty('pointer-events', 'auto', 'important');
        fabButton.style.setProperty('cursor', 'pointer', 'important');
        console.log('✅ Bulldog FAB button clickable');
      }

      if (chatWindow) {
        chatWindow.style.setProperty('pointer-events', 'auto', 'important');
        console.log('✅ Bulldog chat window clickable');
      }
    }

    // STEP 5: Kill any full-screen blocking overlays
    const allFixed = document.querySelectorAll('[style*="position: fixed"], [style*="position: absolute"]');
    allFixed.forEach(el => {
      const computed = window.getComputedStyle(el);
      const rect = el.getBoundingClientRect();

      // If element is full-screen sized and might block
      const isFullWidth = rect.width >= window.innerWidth * 0.9;
      const isFullHeight = rect.height >= window.innerHeight * 0.9;
      const canBlock = computed.pointerEvents !== 'none';

      if ((isFullWidth || isFullHeight) && canBlock && !el.classList.contains('navbar')) {
        // Check if it's not Bulldog or other intentional overlays
        if (!el.className.includes('bulldog') && !el.className.includes('modal') && !el.className.includes('popup')) {
          console.warn('⚠️ Found potential blocking overlay:', el);
          console.log('Element classes:', el.className);
          console.log('Dimensions:', `${Math.round(rect.width)}px × ${Math.round(rect.height)}px`);

          // Make it non-blocking (but keep it visible for debugging)
          el.style.setProperty('pointer-events', 'none', 'important');
          console.log('🔧 Made overlay non-blocking');
        }
      }
    });

    // STEP 6: Verify Bulldog position (ghost div check)
    if (bulldogContainer) {
      const rect = bulldogContainer.getBoundingClientRect();
      const distanceFromRight = window.innerWidth - rect.right;

      if (Math.abs(distanceFromRight - 20) > 10) {
        console.error(`🚨 GHOST DIV DETECTED: Bulldog is ${Math.round(distanceFromRight)}px from right edge (should be ~20px)`);
        console.log('📍 Forcing Bulldog to absolute right corner...');

        // Nuclear option: force position
        bulldogContainer.style.setProperty('right', '20px', 'important');
        bulldogContainer.style.setProperty('left', 'auto', 'important');

        // Re-check
        setTimeout(() => {
          const newRect = bulldogContainer.getBoundingClientRect();
          const newDistance = window.innerWidth - newRect.right;
          console.log('📍 New distance from right:', Math.round(newDistance) + 'px');
        }, 100);
      } else {
        console.log('✅ Bulldog position correct (20px from right edge)');
      }
    }

    console.log('✅ Click Fixer - Complete!');
  }

  // Run on DOM ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', forceButtonClickability);
  } else {
    forceButtonClickability();
  }

  // Also run after a delay to catch dynamic content
  setTimeout(forceButtonClickability, 500);
  setTimeout(forceButtonClickability, 1000);

  // Re-run on any route change (for SPA navigation)
  let lastUrl = location.href;
  new MutationObserver(() => {
    const url = location.href;
    if (url !== lastUrl) {
      lastUrl = url;
      console.log('🔄 Route changed, re-running click fixer...');
      setTimeout(forceButtonClickability, 300);
    }
  }).observe(document, { subtree: true, childList: true });

})();
