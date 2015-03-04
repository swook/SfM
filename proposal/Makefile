all: 3DPhoto_Proposal_Park_Wang.pdf

3DPhoto_Proposal_Park_Wang.pdf: proposal.tex pic1.png pic2.png
	latexmk -pdf -jobname=$(basename $@) $<
	latexmk -c -jobname=$(basename $@)

clean:
	rm -rf 3DPhoto_Proposal_Park_Wang.pdf
