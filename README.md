# Adaptive Control Laboratory

---

# Instructions for the proper use of this repository

This README provides quick and organized instructions for:

* Creating an SSH key
* Adding it to your GitHub account
* Cloning this repository using SSH

---

## 1. Generate a new SSH key

Open a terminal and run:

```bash
type -p ssh >/dev/null || sudo apt install -y openssh-client
```

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

Careful! You should press **Enter** or **y** for all prompts to use the default location. You should also remember the password for your key (if you are not sure, use the password `newkey`).

Then display your public key:

```bash
cat ~/.ssh/id_ed25519.pub
```

and copy it.

---

## 2. Add the SSH key to your GitHub account

In **your GitHub account** go to:

1. `Settings`
2. `SSH and GPG keys`
3. `New SSH key`
4. `Title`: ece-adaptive-control, `Key`: paste your public key
5. Click `Add SSH key`

---

## 3. Create a working directory

```bash
cd [your_path]
```

---

## 4. Clone the project via SSH

```bash
git clone git@github.com:vourouuu/ece-adaptive-control.git
cd ece-adaptive-control
```

Check that it is up to date:

```bash
git pull
```

You should see: `Already up to date`.

---

## 5. Make your changes

Edit any files you need.

View modified files (note: modified files appear in red):

```bash
git status
```

---

## 6. Stage and commit your changes

To stage **all** modified files:

```bash
git add .
git commit -m "Brief description of the changes."
```

> **Note:** If you want to stage only specific files, use:
>
> ```bash
> git add path/to/file1
> git commit -m "Brief description of the changes in file1."
> git add path/to/file2
> git commit -m "Brief description of the changes in file2."
> ...
> ```

---

## 7. Push to the remote repository

```bash
git push
```

---

Done! You are now working with SSH and Git properly.
