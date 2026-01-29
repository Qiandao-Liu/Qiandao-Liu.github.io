document.addEventListener("click", (event) => {
  const link = event.target.closest('a[data-portal="true"]');
  if (!link) return;

  if (event.defaultPrevented || event.button !== 0) return;
  if (event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
  if (link.target && link.target.toLowerCase() === "_blank") return;

  const href = link.getAttribute("href");
  if (!href || href.startsWith("#")) return;

  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
  if (reduceMotion.matches) return;

  event.preventDefault();

  document.body.classList.add("portal-transition--active");

  const timeoutId = window.setTimeout(() => {
    window.location.href = href;
  }, 1100);

  window.addEventListener(
    "pagehide",
    () => {
      window.clearTimeout(timeoutId);
    },
    { once: true }
  );
});
