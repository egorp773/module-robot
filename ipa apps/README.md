# IPA Apps

GitHub Actions writes the iOS build here as:

```text
ipa apps/module.ipa
```

Workflow:

```text
.github/workflows/build-ios-ipa.yml
```

The current workflow builds an unsigned IPA using `flutter build ios --no-codesign`.
That is enough to verify and download the app bundle artifact from GitHub Actions.
For direct installation on a real iPhone, the IPA must be signed with an Apple
Developer certificate and provisioning profile.
