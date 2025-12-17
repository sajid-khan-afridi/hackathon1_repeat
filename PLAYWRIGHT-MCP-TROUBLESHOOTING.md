# Playwright MCP Troubleshooting Guide

## Current Configuration (RECOMMENDED - Already Applied)

Your `.mcp.json` now uses the globally installed Playwright MCP:

```json
{
  "mcpServers": {
    "playwright": {
      "type": "stdio",
      "command": "node",
      "args": [
        "C:\\Users\\uetia\\AppData\\Roaming\\npm\\node_modules\\@playwright\\mcp\\index.js"
      ],
      "env": {}
    }
  }
}
```

**This should work!** ✅

## Testing Steps

1. **Restart Claude Code completely** (close and reopen)
2. Type `/mcp` to verify Playwright appears
3. Test with a simple command:
   ```
   Navigate to https://example.com and get the page title
   ```

## Alternative Configurations (If Above Doesn't Work)

### Option 1: PowerShell with npx

```json
{
  "mcpServers": {
    "playwright": {
      "type": "stdio",
      "command": "powershell.exe",
      "args": [
        "-NoProfile",
        "-Command",
        "npx -y @playwright/mcp@latest"
      ],
      "env": {}
    }
  }
}
```

### Option 2: Full Path to npx.cmd

```json
{
  "mcpServers": {
    "playwright": {
      "type": "stdio",
      "command": "C:\\Program Files\\nodejs\\npx.cmd",
      "args": ["-y", "@playwright/mcp@latest"],
      "env": {}
    }
  }
}
```

### Option 3: Batch Wrapper (Most Reliable)

Use the included `playwright-mcp.bat`:

```json
{
  "mcpServers": {
    "playwright": {
      "type": "stdio",
      "command": "D:\\GitHub Connected\\hackathon1_repeat\\playwright-mcp.bat",
      "args": [],
      "env": {}
    }
  }
}
```

## Common Issues and Solutions

### Issue: "Failed to connect to MCP server"

**Solutions:**
1. Restart Claude Code (must be a FULL restart)
2. Check that Node.js is in your PATH: `node --version`
3. Verify installation: `npm list -g @playwright/mcp`
4. Try the batch wrapper method (Option 3 above)

### Issue: MCP server starts but doesn't respond

**Solutions:**
1. Check Claude Code logs (if available)
2. Try running the command manually to see errors:
   ```cmd
   node "C:\Users\uetia\AppData\Roaming\npm\node_modules\@playwright\mcp\index.js"
   ```
3. Reinstall Playwright MCP:
   ```cmd
   npm uninstall -g @playwright/mcp
   npm install -g @playwright/mcp
   ```

### Issue: "npx not found" or "command not found"

**Solutions:**
1. Install npm/node: https://nodejs.org/
2. Add Node.js to PATH (usually done during installation)
3. Use full path configuration (Option 2)

## Verification Commands

```bash
# Check if everything is installed correctly
node --version          # Should show v22.20.0 or similar
npm --version           # Should show 11.6.2 or similar
npx --version          # Should show version number
npm list -g @playwright/mcp  # Should show installed version

# Test Playwright MCP directly
npx -y @playwright/mcp@latest --version  # Should show version 0.0.52 or similar
```

## What Playwright MCP Can Do

Once connected, you can:
- **Browser Automation**: Navigate, click, type, scroll
- **Data Extraction**: Get page content, extract text, scrape data
- **Screenshots**: Capture page screenshots
- **Testing**: Verify UI elements, test workflows
- **Multi-page**: Handle multiple tabs and windows
- **Network**: Intercept requests, mock responses

## Example Commands

```
Navigate to github.com and search for "playwright"
Take a screenshot of the current page
Click on the "Sign in" button
Get all links on the page
Fill out the contact form with test data
```

## Files Created

- `.mcp.json` - Main MCP configuration (currently configured)
- `playwright-mcp.bat` - Batch wrapper for reliable execution
- `.mcp-alternatives.json` - Alternative configurations for reference
- `PLAYWRIGHT-MCP-TROUBLESHOOTING.md` - This guide

## Support

If issues persist:
1. Check Claude Code documentation: https://github.com/anthropics/claude-code
2. Playwright MCP issues: https://github.com/microsoft/playwright-mcp
3. Verify Windows firewall isn't blocking Node.js
