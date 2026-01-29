// Wind sweep transition - academic to art mode
(function() {
  const TRANSITION_DURATION = 1200;

  document.addEventListener("click", (event) => {
    const link = event.target.closest('a[data-portal="true"]');
    if (!link) return;

    if (event.defaultPrevented || event.button !== 0) return;
    if (event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
    if (link.target && link.target.toLowerCase() === "_blank") return;

    const href = link.getAttribute("href");
    if (!href || href.startsWith("#")) return;

    // Respect reduced motion preference
    const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
    if (reduceMotion.matches) {
      window.location.href = href;
      return;
    }

    event.preventDefault();

    // Trigger wind sweep animation
    document.body.classList.add("wind-sweep--active");

    // Navigate after animation completes
    const timeoutId = window.setTimeout(() => {
      window.location.href = href;
    }, TRANSITION_DURATION);

    window.addEventListener(
      "pagehide",
      () => {
        window.clearTimeout(timeoutId);
      },
      { once: true }
    );
  });
})();
