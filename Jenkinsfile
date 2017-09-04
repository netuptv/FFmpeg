import com.wangyin.parameter.WHideParameterDefinition
import groovy.transform.InheritConstructors
import hudson.AbortException
import hudson.console.ModelHyperlinkNote
import hudson.model.*
import hudson.plugins.git.util.BuildData
import hudson.scm.SubversionTagAction
import jenkins.model.*
import org.jenkinsci.plugins.workflow.job.properties.DisableConcurrentBuildsJobProperty
import org.jenkinsci.plugins.workflow.steps.FlowInterruptedException
import org.jenkinsci.plugins.workflow.support.steps.build.RunWrapper

/**
 *  Unified Jenkinsfile for multibranch builds
 *  Vertion: 0.1
 */


/**
 *  Classes
 */

@InheritConstructors
class FailedBuildException extends Exception {
    public RunWrapper build
    public FailedBuildException(RunWrapper build) {
        super()
        this.build = build
    }
}


/**
 *  Service functions
 */

def getRevision(b)
{
    def gitBuildData = b.getAction(BuildData)
    if (gitBuildData != null) {
        return gitBuildData.lastBuiltRevision
    }
    def svnRevisionState = b.getAction(SubversionTagAction)
    if (svnRevisionState != null) {
        return svnRevisionState.tags.keySet()[0].revision
    }
    return null
}

def getPaMap(ParametersAction pa)
{
    def m = [:]
    if (pa != null) {
        for (p in pa.getParameters()) {
           m[p.name] = p.value
        }
    }
    return m
}

def getPrevBuild()
{
    def curPaMap = getPaMap(currentBuild.rawBuild.getAction(ParametersAction.class))

    def b = currentBuild.rawBuild.parent.lastSuccessfulBuild
    while (b != null) {
        def paMap = getPaMap(b.getAction(ParametersAction.class))
        if (paMap == curPaMap) {
            return b
        }

        b = b.previousSuccessfulBuild
    }

    return null
}

def buildDependency(job_name, parameters)
{
    def build_parameters = []
    for (param in parameters) {
        switch(param.value) {
            case Boolean:
                build_parameters.add(booleanParam(
                    name: param.key,
                    value: param.value))
                break
            case String:
                build_parameters.add(stringParam(
                    name: param.key,
                    value: param.value))
                break
        }
    }

    def job_build = build job: job_name, parameters: build_parameters, propagate: false
    if (job_build.result == "ABORTED") {
        def cause = job_build.rawBuild.getAction(InterruptedBuildAction)
        if (cause != null) {
            throw new AbortException(
                ModelHyperlinkNote.encodeTo(
                    "/" + job_build.rawBuild.url, job_build.fullDisplayName) +
                " interrupted")
        }
    }
    else if (job_build.result == "FAILURE") {
        throw new FailedBuildException(job_build)
    }

    if (job_build.result == "SUCCESS") {
        return job_build
    }

    return null
}

def currentPropsMap()
{
    def prop_list = [:]
    for (prop in currentBuild.rawBuild.project.getAllProperties()) {
        switch (prop) {
            case DisableConcurrentBuildsJobProperty:
                prop_list['disable_concurrent_builds'] = true
                break
            case ParametersDefinitionProperty:
                prop_list['parameters'] = [:]
                for (param in prop.parameterDefinitions) {
                    switch(param) {
                        case BooleanParameterDefinition:
                            prop_list['parameters'][param['name']] = ['type': 'boolean']
                            break
                        case StringParameterDefinition:
                            prop_list['parameters'][param['name']] = ['type': 'string']
                            break
                        case ChoiceParameterDefinition:
                            prop_list['parameters'][param['name']] = [
                                'type': 'choice',
                                'choices': param['choices']]
                            break
                        case WHideParameterDefinition:
                            prop_list['parameters'][param['name']] = [
                                'type': 'hidden',
                                'default_value': param['defaultValue']]
                            break
                    }
                }
                break
        }
    }
    return prop_list
}

def newPropsMap(config)
{
    def prop_list = [:]

    if (!config.containsKey('disable_concurrent_builds') ||
        config['disable_concurrent_builds'])
    {
        prop_list['disable_concurrent_builds'] = true
    }

    prop_list['parameters'] = [:]
    for (p in config['parameters']) {
        switch(p.value['type']) {
            case 'boolean':
                prop_list['parameters'][p.key] = ['type': 'boolean']
                break
            case 'string':
                prop_list['parameters'][p.key] = ['type': 'string']
                break
            case 'choice':
                prop_list['parameters'][p.key] = ['type': 'choice', 'choices': p.value['choices']]
                break
            case 'hidden':
                prop_list['parameters'][p.key] = ['type': 'hidden', 'default_value': p.value['default_value']]
                break
        }
    }

    if (!prop_list['parameters']['wipe']) {
        prop_list['parameters']['wipe'] = ['type': 'boolean']
    }

    return prop_list
}

def setProperties(config)
{
    def prop_list = []

    if (!config.containsKey('disable_concurrent_builds') ||
        config['disable_concurrent_builds'])
    {
        prop_list.add(disableConcurrentBuilds())
    }

    def param_list = []
    for (param in config['parameters']) {
        switch(param.value['type']) {
            case 'boolean':
                param_list.add(booleanParam(
                    name: param.key,
                    defaultValue: param.value['default_value']? true: false,
                    description: param.value['description']? param.value['description']: ''
                ))
                break
            case 'string':
                param_list.add(stringParam(
                    name: param.key,
                    defaultValue: param.value['default_value']? param.value['default_value']: '',
                    description: param.value['description']? param.value['description']: ''
                ))
                break
            case 'choice':
                param_list.add(choiceParam(
                    name: param.key,
                    choices: param.value['choices'].join('\n'),
                    description: param.value['description']? param.value['description']: ''
                ))
                break
            case 'hidden':
                param_list.add([
                    $class: 'WHideParameterDefinition',
                    name: param.key,
                    defaultValue: param.value['default_value']
                ])
                break
        }
    }

    if ( !(config['parameters'] && config['parameters']['wipe']) ) {
        param_list.add(booleanParam(
            name: 'wipe',
            defaultValue: false,
            description: 'wipe workspace before build (clear build)'
        ))
    }

    prop_list.add(parameters(param_list))
    properties(prop_list)
}

def jobFullName(job_name, branch_prefix, branch_name)
{
    def part_list = [job_name]
    if (branch_name) {
        def evaluated_branch = evaluate(/"${branch_name}"/)
        def full_branch
        if (!evaluated_branch.equals('trunk') && branch_prefix)
            full_branch = branch_prefix + '/' + evaluated_branch
        else
            full_branch = evaluated_branch
        part_list.add(full_branch.replace('/', '%2F'))
    }
    return part_list.join('/')
}

def makeCopyParameters(parameters)
{
    copy_parameters = []
    for (param in parameters)
        copy_parameters.add("${param.key}=${param.value}")
    return copy_parameters.join(',')
}


/**
 *  Step functions
 */

def loadConfig()
{
    // Checkout repository and load config
    def config = [:]
    def configfile
    if (env.JOB_NAME.endsWith('-test/' + env.JOB_BASE_NAME))
        configfile = 'Jenkinsconfig.test'
    else
        configfile = 'Jenkinsconfig'

    node ("master") {
        stage ("Checkout on master and load config") {
            checkout scm
            config = readYaml file: configfile
        }
    }
    return config
}

def configureBuild(config)
{
    stage ('Configure job and build names, job properties') {
        // Short branch name
        def branch_short
        if (config['branch_name_prefix'] && env.BRANCH_NAME.startsWith(config['branch_name_prefix'])) {
            branch_short = env.BRANCH_NAME.substring(config['branch_name_prefix'].length())
        }
        else {
            branch_short = env.BRANCH_NAME
        }

        // Set job name
        if (config['job_name']) {
            currentBuild.rawBuild.project.displayName = "${config['job_name']} (${branch_short})"
        }
        else {
            currentBuild.rawBuild.project.displayName = branch_short
        }

        // Update properties
        // Check only important for current build changes and abort build if found
        def current_props = currentPropsMap()
        def new_props = newPropsMap(config)
        setProperties(config)

        if (current_props != new_props) {
            println 'Properties changed. Abort build. Start build again with new parameters'
            def cause = currentBuild.rawBuild.getCause(Cause.class)
            while (cause instanceof Cause.UpstreamCause) cause = cause.upstreamCauses[0]
            currentBuild.rawBuild.executor
                .interrupt(
                    Result.ABORTED,
                    new CauseOfInterruption.UserInterruption(cause.userId))
            currentBuild.result = 'ABORTED'
            return
        }

        // Get build number
        if (config['build_number_provider']) {
            def bn_build = build job: config['build_number_provider']
            env.BUILD_NUMBER = bn_build.number
        }

        // Set build name
        if (config['build_name']) {
            currentBuild.displayName = evaluate(/"${config['build_name']}"/)
        }
        else {
            currentBuild.displayName = "#${env.BUILD_NUMBER}"
        }
    }
}

def buildAllDependencies(config)
{
    // Build dependencies and get builds for copy artifacts
    def build_result_list = []
    if (config['dependencies']) {
        stage ('Build dependencies') {
            def parallel_jobs = [failFast: true]
            for (job_name in config['dependencies'].keySet()) {
                def job_options = config['dependencies'][job_name]
                def job_full_name = jobFullName(job_name, job_options['branch_prefix'], job_options['branch'])
                def parameters = job_options['parameters']
                parallel_jobs[job_name] = {
                    return buildDependency(job_full_name, parameters)
                }
            }
            try {
                build_result_list = parallel(parallel_jobs)
            }
            catch (AbortException e) {
                currentBuild.result="ABORTED"
                throw e
            }
            catch (FailedBuildException e) {
                currentBuild.result = 'FAILURE'
                println "Build failed in " + ModelHyperlinkNote.encodeTo(
                        "/" + e.build.rawBuild.url, e.build.fullDisplayName)
                throw e
            }
        }
    }
    return build_result_list
}

def prepareWorkspace()
{
    // Clean up
    stage('Clean up') {
        if (params['wipe'])
            sh 'ls -A | xargs -r rm -rf'
        else
            sh 'rm -rf jenkins/out'
    }

    // Checkout if not in master (on master checked out earlier) or after wipe
    if (env.NODE_NAME != 'master') {
        stage ("Checkout on node ${env.NODE_NAME}") {
            checkout scm
        }
    }
    else {
        stage ("Checkout if need (after wipe)") {
            if (params['wipe']) {
                checkout scm
            }
        }
    }
}

def copyArtifacts(config, build_result_list)
{
    def artifact_list = [:]
    for(field in ['dependencies', 'additional_artifacts']) {
        if (config[field]) artifact_list += config[field]
    }
    if (artifact_list) {
        stage("Copy artifacts") {
            for(job_name in artifact_list.keySet()) {
                def job_options = artifact_list[job_name]
                try {
                    if (build_result_list[job_name]) {
                        step([
                            $class: "CopyArtifact",
                            projectName: jobFullName(job_name, job_options['branch_prefix'], job_options['branch']),
                            selector: [
                                $class: "SpecificBuildSelector",
                                buildNumber: String.valueOf(build_result_list[job_name].number)]])
                    }
                    else {
                        step([
                            $class: "CopyArtifact",
                            projectName: jobFullName(job_name, job_options['branch_prefix'], job_options['branch']),
                            parameters: makeCopyParameters(job_options['parameters']),
                            selector: [
                                $class: "StatusBuildSelector",
                                stable: false]])
                    }
                }
                catch (any) {
                    println "Can't copy artifacts from job ${job_name}"
                    currentBuild.result = 'FAILURE'
                    throw any
                }
            }
        }
    }

    return artifact_list
}

def hasChanges(config, artifact_list)
{
    return stage ("Check changes") {
        def prev_build = getPrevBuild()
        if (prev_build == null) {
            println "Can't find previous successful build"
            return true
        }
        if (getRevision(currentBuild.rawBuild) == getRevision(prev_build)) {
            println 'Current and previous sucsessful build revisions match'
            def build_num = prev_build.number
            prev_build = null

            if (artifact_list) {
                def build_info_filename = "jenkins/out/${config['out_dir']}/build.info"
                try {
                    step([
                        $class: 'CopyArtifact',
                        filter: build_info_filename,
                        projectName: env.JOB_NAME,
                        selector: [
                            $class: 'SpecificBuildSelector',
                            buildNumber: String.valueOf(build_num)]])
                }
                catch (any) {
                    println "Can't load previous build build.info"
                    return true
                }
                def prev_build_info = readProperties file: build_info_filename
                for (job_name in artifact_list.keySet()) {
                    def job_options = artifact_list[job_name]
                    def artifact_build_info = readProperties file: 'jenkins/out/' + job_options['build_info']
                    for (prop in artifact_build_info.keySet()) {
                        if (artifact_build_info[prop] != prev_build_info[config['build_info_prefix'] + '_' + prop]) {
                            println "Found change in job ${job_name}"
                            return true
                        }
                    }
                }
            }
            println 'No changes found, abort build'
            currentBuild.result = 'ABORTED'
            return false
        }
        return true
    }
}

def buildStep(config)
{
    stage ('Build') {
        def opt_list = []
        for (p in config['parameters']) {
            if (p.key == 'wipe') continue
            def opt_name = p.key.replace('_', '-')
            switch (p.value['type']) {
                case 'boolean':
                    if (params[p.key])
                        opt_list.add('--' + opt_name)
                    break
                default:
                    opt_list.add('--' + opt_name + '=' + params[p.key])
                    break
            }
        }
        sh "mkdir -p jenkins"

        def script_list
        if (config['script'] instanceof String)
            script_list = [config['script']]
        else 
            script_list = config['script']

        for (scr in script_list)
            sh "cd jenkins; ../${scr} ${opt_list.join(' ')}"
    }
}

def saveArtifacts(config)
{
    stage ('Save artifacts') {
        def archive_dir = currentBuild.rawBuild.pickArtifactManager().root().toURI().path
        def wildcard
        if (config['artifacts'])
            wildcard = "jenkins/out/${config['out_dir']}/${config['artifacts']}"
        else
            wildcard = "jenkins/out/${config['out_dir']}"
        if (env.NODE_NAME == 'master') {
            sh "mkdir -p ${archive_dir}"
            sh "tar cf - ${wildcard} | tar xf - -C ${archive_dir}"
        }
        else {
            sh "tar cf jenkins/artifacts.tar ${wildcard}"
            archiveArtifacts 'jenkins/artifacts.tar'
            node ('master') {
                ws(archive_dir) {
                    sh "tar xf jenkins/artifacts.tar && rm -f jenkins/artifacts.tar"
                }
            }
            sh 'rm -f jenkins/artifacts.tar'
        }
    }
}

def publishJunit(config)
{
    if (config['junit']) {
        stage ('Publish test results') {
            junit "jenkins/out/${config['out_dir']}/${config['junit']}"
        }
    }
}


/*
 *  Main
 */

def config = [:]
try {
    config = loadConfig()
    configureBuild(config)
    if (currentBuild.result == 'ABORTED') return
    def build_result_list = buildAllDependencies(config)
    node (config['node']) {
        prepareWorkspace()
        def artifact_list = copyArtifacts(config, build_result_list)
        if (hasChanges(config, artifact_list)) {
            buildStep(config)
            saveArtifacts(config)
            publishJunit(config)
        }
    }
}
catch (FlowInterruptedException e) {
    println "Build interrupted"
}
catch (any) {
    println any
    currentBuild.result = 'FAILURE'
    throw any
}
finally {
    if (config.emails) {
        node() {
            step([
                $class: 'Mailer',
                notifyEveryUnstableBuild: false,
                recipients: config.emails.join(' '),
                sendToIndividuals: false
            ])
        }
    }
}
