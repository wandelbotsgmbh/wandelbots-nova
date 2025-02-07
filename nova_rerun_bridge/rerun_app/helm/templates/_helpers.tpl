{{/*
Expand the name of the chart.
*/}}
{{- define "rerun-nova.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" }}
{{- end }}

{{/*
Create a default fully qualified app name.
We truncate at 63 chars because some Kubernetes name fields are limited to this (by the DNS naming spec).
If release name contains chart name it will be used as a full name.
*/}}
{{- define "rerun-nova.fullname" -}}
{{- if .Values.fullnameOverride }}
{{- .Values.fullnameOverride | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- $name := default .Chart.Name .Values.nameOverride }}
{{- if contains $name .Release.Name }}
{{- .Release.Name | trunc 63 | trimSuffix "-" }}
{{- else }}
{{- printf "%s-%s" .Release.Name $name | trunc 63 | trimSuffix "-" }}
{{- end }}
{{- end }}
{{- end }}

{{/*
Create chart name and version as used by the chart label.
*/}}
{{- define "rerun-nova.chart" -}}
{{- printf "%s-%s" .Chart.Name .Chart.Version | replace "+" "_" | trunc 63 | trimSuffix "-" }}
{{- end }}

{{/*
Common labels
*/}}
{{- define "rerun-nova.labels" -}}
helm.sh/chart: {{ include "rerun-nova.chart" . }}
{{ include "rerun-nova.selectorLabels" . }}
{{- if .Chart.AppVersion }}
app.kubernetes.io/version: {{ .Chart.AppVersion | quote }}
{{- end }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
{{- end }}

{{/*
Selector labels
*/}}
{{- define "rerun-nova.selectorLabels" -}}
app.kubernetes.io/name: {{ include "rerun-nova.name" . }}
app.kubernetes.io/instance: {{ .Release.Name }}
{{- end }}

{{/*
Create the name of the service account to use
*/}}
{{- define "rerun-nova.serviceAccountName" -}}
{{- if .Values.serviceAccount.create }}
{{- default (include "rerun-nova.fullname" .) .Values.serviceAccount.name }}
{{- else }}
{{- default "default" .Values.serviceAccount.name }}
{{- end }}
{{- end }}

{{/*
This function returns either "nvidia", "intel" or "none" based on the available GPU resources on the node.
More specifically, this will return the very last gpu found in the labels of all nodes.
*/}}
{{- define "rerun-nova.nodegpu" -}}
{{- $out := "none" -}}
{{- range $index, $node := (lookup "v1" "Node" "" "").items }}
    {{- range $k, $v := $node.status.allocatable }}
        {{- if and (eq $k "nvidia.com/gpu") (gt ($v | int) 0)}}
            {{- $out = "nvidia" -}}
        {{- else if and (eq $k "gpu.intel.com/i915") (gt ($v | int) 0) }}
            {{- $out = "intel" -}}
        {{- end }}
    {{- end }}
{{- end }}
{{- $out }}
{{- end }}

{{/*
This function tries to get the ip for the node. Works only for single nodes.
Might fail if there is no node (e.g. test templating).
*/}}
{{- define "rerun-nova.nodeip" -}}
{{- $nodes := lookup "v1" "Node" "" "" -}}
{{- if gt (len $nodes) 0 -}}
    {{- $node := index $nodes.items 0 }}
    {{- if $node }}
        {{- range $k, $v := $node.status.addresses }}
            {{- if eq $v.type "InternalIP" -}}
                {{- $v.address -}}
                {{- break -}}
            {{- end }}
        {{- end }}
    {{- end }}
{{- end }}
{{- end }}
