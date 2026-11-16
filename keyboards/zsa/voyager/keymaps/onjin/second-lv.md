# Deploy `some.html` to `taipo.second.lv` from the Cloudflare CLI

This tutorial covers deploying a single HTML file to a Cloudflare Pages project and attaching it as a subdomain of an existing Cloudflare zone (`second.lv`).

## Prerequisites

- A Cloudflare account with the zone `second.lv` active.
- A narrow-scoped API token with:
  - **Account** → `Cloudflare Pages` → `Edit`
  - **Zone** → `DNS` → `Edit` (limited to `second.lv`)
- Wrangler installed: `npm install -g wrangler`
- Environment variables set in your shell:

```sh
export CLOUDFLARE_API_TOKEN="<your-token>"
export CLOUDFLARE_ACCOUNT_ID="a14c0be30ee59f3ed7e70ef3e91c26b4"
```

---

## Step 1: Prepare the upload directory

Pages requires a directory to upload. Rename `some.html` to `index.html` inside a project folder.

```sh
mkdir -p taipo-site
cp some.html taipo-site/index.html
```

---

## Step 2: Create the Pages project

```sh
npx wrangler pages project create taipo --production-branch main
```

If the project name `taipo` is already taken, Wrangler will append random characters. You can use a different unique name if needed.

---

## Step 3: Deploy the site

```sh
npx wrangler pages deploy ./taipo-site --project-name taipo --branch main
```

After deployment, the site is available at the default `*.pages.dev` URL (for example, `taipo.pages.dev`).

---

## Step 4: Attach `taipo.second.lv`

Wrangler does not manage Pages custom domains directly. Use the Cloudflare API to add the subdomain. Because `second.lv` is already a Cloudflare zone on your account, Cloudflare automatically creates the proxied CNAME record and provisions the certificate.

```sh
curl -s -X POST "https://api.cloudflare.com/client/v4/accounts/$CLOUDFLARE_ACCOUNT_ID/pages/projects/taipo/domains" \
  -H "Authorization: Bearer $CLOUDFLARE_API_TOKEN" \
  -H "Content-Type: application/json" \
  --data '{"name":"taipo.second.lv"}'
```

---

## Step 5: Verify the custom domain is active

```sh
curl -s "https://api.cloudflare.com/client/v4/accounts/$CLOUDFLARE_ACCOUNT_ID/pages/projects/taipo/domains" \
  -H "Authorization: Bearer $CLOUDFLARE_API_TOKEN"
```

Wait until the `status` for `taipo.second.lv` changes to `active`. Then open:

```
https://taipo.second.lv
```

---

## Optional: remove the custom domain later

```sh
curl -s -X DELETE "https://api.cloudflare.com/client/v4/accounts/$CLOUDFLARE_ACCOUNT_ID/pages/projects/taipo/domains/taipo.second.lv" \
  -H "Authorization: Bearer $CLOUDFLARE_API_TOKEN"
```

Then manually remove the leftover `taipo` CNAME record in the `second.lv` DNS zone if it remains.
